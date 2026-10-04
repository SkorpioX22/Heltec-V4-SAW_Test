/*
  LilyGO T3-S3 (SX1262) - v2 round-trip reference receiver + talkback TX
  Part of: Heltec V4 RSSI / SNR Test Procedure (T3S3 reference receiver)

  Role: receives the numbered packets from the Heltec V4 R2 / R8 transmitters,
        immediately transmits a talkback packet, waits for the heltec's ack
        and logs every packet as CSV including the ack values.

  Arduino IDE settings (esp32 core 3.3.x):
    Board           : "LilyGo T3-S3"
    Revision        : "Radio_SX1262"
    USB CDC On Boot : Enabled
    PSRAM           : Enabled (QSPI)
    Upload Speed    : 921600

  Round trip inside every 1 s slot (all packets fixed 32 bytes):
    1. R2/R8 -> this board : data     "<id>,<seq>"
    2. this board -> R2/R8 : talkback "T3,<seq>"              (22 dBm)
    3. R2/R8 -> this board : ack      "AK,<seq>,<rssi>,<snr>"
       = the heltec's measured RSSI/SNR of OUR talkback packet
  The ack window is 200 ms; on timeout the CSV line ends with ",-,-".

  Data line format (fed to t3s3_logger_v2.py):
      R2,500m,000001,-71,8.25,-65,7.50
      fields: transmitter, distance, packet, rssi_dbm, snr_db,
              tb_rssi_dbm (heltec's view of our talkback), tb_snr_db
  Lines beginning with '#' are comments or summaries, not data.

  Serial commands (115200 baud):
      d<number>  set test distance in metres, 0..9999 (e.g. "d500" -> 500m)
      r          reset statistics (new run)
      s          print statistics summary now
      h or ?     help
*/

#include <RadioLib.h>
#include <stdlib.h>
#include <string.h>
#include "SSD1306Wire.h"

// ==================== T3-S3 SX1262 pin mapping ========================
#define LORA_NSS   7
#define LORA_DIO1  33
#define LORA_RST   8
#define LORA_BUSY  34
#define LORA_SCK   5
#define LORA_MISO  3
#define LORA_MOSI  6
#define LED_PIN    37     // T3-S3 onboard LED

// ==================== OLED (T3-S3 0.96" SSD1306) ======================
#define OLED_SDA   18
#define OLED_SCL   17
#define OLED_FLIP  true  // set true if the screen appears upside down
SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL, GEOMETRY_128_64);

// ====================== radio settings (baseline) ======================
// MUST be identical on R2, R8 and this receiver.
#define FREQ_MHZ        923.2f    // AS923 channel, Singapore 915-928 MHz
                                  // (923.4 / 923.6 ... 924.6 also valid)
#define BANDWIDTH_KHZ   125.0f    // section 4: 125 kHz
#define SPREAD_FACTOR   7         // section 4: SF7 (repeat runs with other SFs)
#define CODING_RATE     5         // section 4: 4/5
#define TX_POWER_DBM    22        // max power - used for the talkback packet
#define PREAMBLE_LEN    8
#define SYNC_WORD       RADIOLIB_SX126X_SYNC_WORD_PRIVATE   // 0x12

// ======================== test parameters =============================
#define PACKETS_PER_RUN 25        // rounds per run
#define PAYLOAD_LEN      32       // fixed payload length (all three packets)
#define ACK_WAIT_MS     200       // window for the heltec's ack after talkback

// ============================== radio =================================
SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

// packet-ready flag set from the SX1262 interrupt
volatile bool receivedFlag = false;
void IRAM_ATTR onPacket(void) { receivedFlag = true; }

// ========================== run statistics ============================
char     runLabel[8] = "";
char     distStr[8]  = "XXX";            // "010m" once set with "d10"
bool     seen[PACKETS_PER_RUN + 1];
uint32_t totalPkts   = 0;                // every valid packet received
uint32_t uniqueCount = 0;                // unique packet numbers 1..100
uint32_t lastSeq     = 0;
float    rssiSum = 0, snrSum = 0;
float    rssiMin = 0, rssiMax = 0;
float    snrMin  = 0, snrMax  = 0;
bool     haveStats   = false;
uint32_t ledOffAt    = 0;
float    lastRssi    = 0;                 // of the most recent packet
float    lastSnr     = 0;
uint32_t tbOk        = 0;                 // talkbacks acked by the heltec
uint32_t tbLost      = 0;                 // talkbacks not acked
char     tbState[8]  = "-";               // "ok" / "LOST" for the OLED

// ============================== OLED ==================================
void oledInit() {
  display.init();
  display.setContrast(255);
#if OLED_FLIP
  display.flipScreenVertically();
#endif
}

void oledDraw() {
  char line[40];

  display.clear();

  // transmitter label + distance
  display.setFont(ArialMT_Plain_16);
  snprintf(line, sizeof(line), "%s %s",
           runLabel[0] ? runLabel : "--", distStr);
  display.drawString(0, 0, line);

  display.setFont(ArialMT_Plain_10);
  int miss = (int)(PACKETS_PER_RUN - uniqueCount);
  String loss = String(100.0f * (float)miss / (float)PACKETS_PER_RUN, 1);
  snprintf(line, sizeof(line), "%03lu/%d  loss %s%%",
           (unsigned long)uniqueCount, PACKETS_PER_RUN, loss.c_str());
  display.drawString(0, 20, line);

  String rssi = haveStats ? String(lastRssi, 0) : String("--");
  String snr  = haveStats ? String(lastSnr, 2)  : String("--");
  display.drawString(0, 32, "RSSI " + rssi + "  SNR " + snr);

  // talkback tally + state (transmission status)
  snprintf(line, sizeof(line), "TB %03lu/%03lu  %s",
           (unsigned long)tbOk, (unsigned long)(tbOk + tbLost), tbState);
  display.drawString(0, 44, line);

  uint8_t pct = (uint8_t)((uniqueCount * 100UL) / PACKETS_PER_RUN);
  display.drawProgressBar(0, 54, 128, 8, pct);

  display.display();
}

// ========================= serial commands ============================
char     cmdBuf[24];
uint8_t  cmdLen = 0;
uint32_t cmdLastMs = 0;

void printHelp() {
  Serial.println(F("# commands: d<number> = distance in m (e.g. d10)"));
  Serial.println(F("#           r = reset stats, s = summary, h = help"));
}

void printConfig() {
  Serial.print(F("# freq="));            Serial.print(FREQ_MHZ, 1);
  Serial.print(F("MHz bw="));            Serial.print(BANDWIDTH_KHZ, 0);
  Serial.print(F("kHz sf="));            Serial.print(SPREAD_FACTOR);
  Serial.print(F(" cr=4/"));             Serial.print(CODING_RATE);
  Serial.print(F(" power="));            Serial.print(TX_POWER_DBM);
  Serial.print(F("dBm preamble="));      Serial.print(PREAMBLE_LEN);
  Serial.print(F(" sync=0x12 packets="));Serial.print(PACKETS_PER_RUN);
  Serial.print(F(" distance="));         Serial.print(distStr);
  Serial.println();
}

void resetStats() {
  memset(seen, 0, sizeof(seen));
  totalPkts   = 0;
  uniqueCount = 0;
  lastSeq     = 0;
  rssiSum = snrSum = 0;
  rssiMin = rssiMax = snrMin = snrMax = 0;
  haveStats = false;
  tbOk   = 0;
  tbLost = 0;
  strncpy(tbState, "-", sizeof(tbState) - 1);
  tbState[sizeof(tbState) - 1] = '\0';
}

void startRun(const char *label) {
  resetStats();
  strncpy(runLabel, label, sizeof(runLabel) - 1);
  runLabel[sizeof(runLabel) - 1] = '\0';
  Serial.print(F("# NEW RUN tx="));
  Serial.print(runLabel);
  Serial.print(F(" dist="));
  Serial.println(distStr);
  oledDraw();
}

void printSummary() {
  float loss = 100.0f * (float)(PACKETS_PER_RUN - uniqueCount) /
               (float)PACKETS_PER_RUN;
  Serial.print(F("# SUMMARY tx="));     Serial.print(runLabel[0] ? runLabel : "?");
  Serial.print(F(" dist="));            Serial.print(distStr);
  Serial.print(F(" expected="));        Serial.print(PACKETS_PER_RUN);
  Serial.print(F(" recv="));            Serial.print(uniqueCount);
  Serial.print(F(" total="));           Serial.print(totalPkts);
  Serial.print(F(" loss_pct="));        Serial.print(loss, 1);
  Serial.print(F(" tb_ok="));           Serial.print(tbOk);
  Serial.print(F(" tb_lost="));         Serial.print(tbLost);
  if (haveStats) {
    Serial.print(F(" avg_rssi="));      Serial.print(rssiSum / uniqueCount, 1);
    Serial.print(F(" min_rssi="));      Serial.print(rssiMin, 1);
    Serial.print(F(" max_rssi="));      Serial.print(rssiMax, 1);
    Serial.print(F(" avg_snr="));       Serial.print(snrSum / uniqueCount, 2);
    Serial.print(F(" min_snr="));       Serial.print(snrMin, 2);
    Serial.print(F(" max_snr="));       Serial.print(snrMax, 2);
  }
  Serial.println();
}

void setDistance(const char *s) {
  long m = strtol(s, NULL, 10);
  if (m < 0 || m > 9999) {
    Serial.println(F("# distance must be 0..9999 m"));
    return;
  }
  snprintf(distStr, sizeof(distStr), "%03ldm", m);
  Serial.print(F("# distance set to "));
  Serial.println(distStr);
}

void processCommand() {
  cmdBuf[cmdLen] = '\0';
  if (cmdLen == 0) return;

  char c = cmdBuf[0];
  if (c == 'd' || c == 'D') {
    setDistance(cmdBuf + 1);
  } else if (c >= '0' && c <= '9') {
    setDistance(cmdBuf);
  } else if (c == 'r' || c == 'R') {
    startRun(runLabel[0] ? runLabel : "???");
    Serial.println(F("# stats reset"));
  } else if (c == 's' || c == 'S') {
    printSummary();
  } else if (c == 'h' || c == 'H' || c == '?') {
    printHelp();
  } else {
    Serial.print(F("# unknown command: "));
    Serial.println(cmdBuf);
  }
  cmdLen = 0;
  oledDraw();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) processCommand();
    } else if (cmdLen == 0 &&
               (c == 'r' || c == 'R' || c == 's' || c == 'S' ||
                c == 'h' || c == 'H' || c == '?')) {
      cmdBuf[0] = c;                 // single letter commands act immediately
      cmdLen = 1;
      processCommand();
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
      cmdLastMs = millis();
    }
  }

  // also accept "d10" sent without a line ending, after a short pause
  if (cmdLen > 0 && (int32_t)(millis() - cmdLastMs) > 400) processCommand();
}

void halt(const __FlashStringHelper *msg) {
  Serial.print(F("# FATAL: "));
  Serial.println(msg);
  while (true) delay(10);
}

// ---- v2 round trip ----------------------------------------------------
// talkback packet "T3,<seq>" padded to 32 bytes; the heltec measures its
// RSSI / SNR and sends them back in the ack
bool sendTalkback(uint32_t seq) {
  char tb[PAYLOAD_LEN + 1];
  snprintf(tb, sizeof(tb), "T3,%06lu", (unsigned long)seq);
  size_t n = strlen(tb);
  for (size_t i = n; i < PAYLOAD_LEN; i++) tb[i] = '.';
  tb[PAYLOAD_LEN] = '\0';

  int st = radio.transmit((uint8_t *)tb, PAYLOAD_LEN);
  // the TX_DONE irq pulses DIO1 and would fake a received packet
  receivedFlag = false;
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print(F("# talkback TX error code="));
    Serial.println(st);
    return false;
  }
  return true;
}

// wait up to ACK_WAIT_MS for "AK,<seq>,<rssi>,<snr>" from the heltec
bool waitAck(uint32_t seq, float *rssi, float *snr) {
  int ar = radio.startReceive();
  if (ar != RADIOLIB_ERR_NONE) return false;

  uint32_t t0 = millis();
  while (!receivedFlag && (int32_t)(millis() - t0) < ACK_WAIT_MS) delay(1);
  if (!receivedFlag) return false;
  receivedFlag = false;

  size_t len = radio.getPacketLength();
  if (len == 0 || len > 63) return false;
  char ak[64];
  if (radio.readData((uint8_t *)ak, len) != RADIOLIB_ERR_NONE) return false;
  ak[len] = '\0';

  if (strncmp(ak, "AK,", 3) != 0) return false;
  unsigned long s = 0;
  if (sscanf(ak + 3, "%lu,%f,%f", &s, rssi, snr) != 3) return false;
  return (uint32_t)s == seq;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  oledInit();

  // dedicated SPI bus for the SX1262 (pins are not the variant defaults)
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  Serial.println(F("# T3S3 v2 reference receiver + talkback"));
  Serial.print(F("# SX1262 init ... "));
  int state = radio.begin(FREQ_MHZ, BANDWIDTH_KHZ, SPREAD_FACTOR, CODING_RATE,
                          SYNC_WORD, TX_POWER_DBM, PREAMBLE_LEN);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code "));
    Serial.println(state);
    halt(F("radio.begin() - check wiring / board settings"));
  }
  Serial.println(F("ok"));

  // explicit header, CRC is already enabled by radio.begin()
  radio.explicitHeader();

  // callback fired by the SX1262 DIO1 interrupt on a received packet
  radio.setPacketReceivedAction(onPacket);

  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("# startReceive failed, code "));
    Serial.println(state);
    halt(F("startReceive()"));
  }

  startRun("???");
  Serial.println(F("# listening - start the transmitter now"));
  printConfig();
  printHelp();
  Serial.println(F("# CSV: transmitter,distance,packet,rssi_dbm,snr_db,tb_rssi_dbm,tb_snr_db"));
}

void handlePacket() {
  size_t len = radio.getPacketLength();          // read BEFORE readData()
  if (len == 0 || len > 63) {
    radio.startReceive();
    return;
  }

  char buf[64];
  int state = radio.readData((uint8_t *)buf, len);
  float rssi = radio.getRSSI();                  // packet RSSI, dBm
  float snr  = radio.getSNR();                   // packet SNR, dB

  buf[len] = '\0';
  digitalWrite(LED_PIN, HIGH);
  ledOffAt = millis() + 50;

  if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println(F("# CRC error (packet dropped)"));
    radio.startReceive();
    return;
  }
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("# readData error code="));
    Serial.println(state);
    radio.startReceive();
    return;
  }

  // payload is "<label>,<seq>" padded with '.'
  char label[8] = "?";
  uint32_t seq  = 0;
  char *comma = strchr(buf, ',');
  if (comma) {
    *comma = '\0';
    strncpy(label, buf, sizeof(label) - 1);
    label[sizeof(label) - 1] = '\0';
    seq = strtoul(comma + 1, NULL, 10);
  }

  // internal protocol packets ("AK,..."/"T3,...") must never be mistaken
  // for a data packet (a late ack would otherwise fake a new run)
  if (seq < 1 || seq > PACKETS_PER_RUN ||
      strcmp(label, "AK") == 0 || strcmp(label, "T3") == 0) {
    Serial.print(F("# unexpected packet: "));
    Serial.println(buf);
    radio.startReceive();
    return;
  }

  // a new run starts on a label change, on packet 1, or after a sequence gap
  if (strcmp(label, runLabel) != 0 ||
      (seq == 1 && totalPkts > 0) ||
      (seq + 5 < lastSeq)) {
    startRun(label);
  }

  totalPkts++;
  if (!seen[seq]) {
    seen[seq] = true;
    uniqueCount++;
    if (!haveStats) {
      rssiMin = rssiMax = rssi;
      snrMin  = snrMax  = snr;
      haveStats = true;
    } else {
      if (rssi < rssiMin) rssiMin = rssi;
      if (rssi > rssiMax) rssiMax = rssi;
      if (snr  < snrMin)  snrMin  = snr;
      if (snr  > snrMax)  snrMax  = snr;
    }
    rssiSum += rssi;
    snrSum  += snr;
  }
  lastSeq = seq;
  lastRssi = rssi;
  lastSnr  = snr;

  // ---- v2 round trip: talkback, then the heltec's ack ----
  float tbRssi = 0, tbSnr = 0;
  bool acked = sendTalkback(seq) && waitAck(seq, &tbRssi, &tbSnr);
  if (acked) {
    tbOk++;
    strncpy(tbState, "ok", sizeof(tbState) - 1);
  } else {
    tbLost++;
    strncpy(tbState, "LOST", sizeof(tbState) - 1);
  }
  tbState[sizeof(tbState) - 1] = '\0';

  // transmitter,distance,packet,RSSI,SNR,tb_rssi,tb_snr
  char seqStr[8];
  snprintf(seqStr, sizeof(seqStr), "%06lu", (unsigned long)seq);
  Serial.print(label);      Serial.print(',');
  Serial.print(distStr);    Serial.print(',');
  Serial.print(seqStr);     Serial.print(',');
  Serial.print(rssi, 0);    Serial.print(',');
  Serial.print(snr, 2);
  if (acked) {
    Serial.print(',');  Serial.print(tbRssi, 0);
    Serial.print(',');  Serial.print(tbSnr, 2);
  } else {
    Serial.print(F(",-,-"));
  }
  Serial.println();

  if (uniqueCount == PACKETS_PER_RUN) printSummary();

  radio.startReceive();      // back to listening for the next packet
  oledDraw();
}

void loop() {
  if (receivedFlag) {
    receivedFlag = false;
    handlePacket();
  }

  if (ledOffAt && (int32_t)(millis() - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);
    ledOffAt = 0;
  }

  handleSerial();
}
