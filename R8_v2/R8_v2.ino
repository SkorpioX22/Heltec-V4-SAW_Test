/*
  Heltec WiFi LoRa 32 V4 R8 (ESP32-S3R8) - v2 talkback test TX
  Part of: Heltec V4 RSSI / SNR Test Procedure (T3S3 reference receiver)
  NOTE: the R2 (ESP32-S3R2) board uses ..\R2_v2\R2_v2.ino

  Role: round-trip test transmitter, 1 packet/s, 25 packets per run.
        Flash THIS sketch to the R8 board only (DEVICE_ID "R8").

  Round trip inside every 1 s slot (all packets fixed 32 bytes):
    1. R2/R8  -> T3S3   data    "<id>,<seq>"          (seq 000001..000025)
    2. T3S3   -> R2/R8  talkback "T3,<seq>"
    3. R2/R8  -> T3S3   ack     "AK,<seq>,<rssi>,<snr>"
       = our measured RSSI/SNR of the talkback packet, logged by the T3S3
         into the CSV (columns <d>m_TB_RSSI / <d>m_TB_SNR).
  If step 2 never happens inside the 200 ms window, the packet counts as
  talkback LOST (no ack is sent).

  Radio: SX1262 controlled with RadioLib, AS923 / Singapore (915-928 MHz),
         ~22 dBm radiated (11 dBm register + KCT8103L FEM ~ +11 dB).

  Arduino IDE settings (esp32 core 3.3.x, no Heltec board package required):
    Board           : "ESP32S3 Dev Module"
    USB CDC On Boot : Enabled
    Flash Size      : 16MB (128Mb)
    PSRAM           : Disabled       - the R8 has 8MB octal PSRAM (OPI), not used
                                       by this sketch; GPIO40/GPIO45 stay free
    Partition Scheme: Default 4MB with spiffs
    Upload Speed    : 921600

  Serial log: disabled by default (SERIAL_DBG 0). Status + talkback tally
              are on the OLED; CSV logging is done by the T3S3.
*/

#include <RadioLib.h>
#include <Wire.h>
#include "SSD1306Wire.h"

// Set to 1 to re-enable the serial console (TX,XXX lines, 'g'/'h' commands,
// panic backtraces visible). 0 = no serial I/O at all.
#define SERIAL_DBG 0
#if SERIAL_DBG
  #define sprint(...)   sprint(__VA_ARGS__)
  #define sprintln(...) sprintln(__VA_ARGS__)
#else
  #define sprint(...)
  #define sprintln(...)
#endif

// ========================= device identity ============================
#define DEVICE_ID "R8"
#define BUILD_MARK "v2"    // shown top-right on the OLED = this build is running

// ===================== Heltec WiFi LoRa 32 V4 pins =====================
// ESP32-S3R8 + SX1262 + KCT8103L front end, pin compatible with the V3.
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_RST   12
#define LORA_BUSY  13
#define LORA_SCK   9
#define LORA_MISO  11
#define LORA_MOSI  10
#define FEM_EN     2      // KCT8103L CSD - front end enable, must stay HIGH
#define VFEM_LDO   7      // FEM LDO power enable (Meshtastic: LORA_PA_POWER 7).
                          // Without this the FEM has no supply and the board
                          // radiates only leakage (~ -78 dBm at point-blank)
#define FEM_CTX    5      // KCT8103L CTX: HIGH = transmit PA mode
                          // (CPS is wired to DIO2 and switches automatically)
#define BOOT_BTN   0      // push to (re)start a run when idle

// ==================== OLED (Heltec V4 0.96" SSD1315) ==================
// V4 wiring: SDA = GPIO17, SCL = GPIO18, RST = GPIO21 (same on R2 and R8)
#define OLED_SDA   17
#define OLED_SCL   18
#define OLED_RST   21
#define OLED_ADDR  0x3C   // 0x3D on some panels

// OLED + LoRa antenna boost power rail (Vext), active LOW.
// Heltec support confirms: on the V4 R8, Vext is GPIO40 (R2 uses GPIO36).
// Leaving it floating = dim display and degraded RF (weak RSSI).
#define VEXT_PIN   40

// 400 kHz instead of the library's 700 kHz default - more reliable over
// the V4 B2B screen connector
SSD1306Wire display(OLED_ADDR, OLED_SDA, OLED_SCL, GEOMETRY_128_64,
                    I2C_ONE, 400000);

// ====================== radio settings (baseline) ======================
// MUST be identical on R2, R8 and the T3S3 receiver.
#define FREQ_MHZ        923.2f    // AS923 channel, Singapore 915-928 MHz
                                  // (923.4 / 923.6 ... 924.6 also valid)
#define BANDWIDTH_KHZ   125.0f    // section 4: 125 kHz
#define SPREAD_FACTOR   7         // section 4: SF7 (repeat runs with other SFs)
#define CODING_RATE     5         // section 4: 4/5
// Radiated power (procedure amended to 22 dBm):
// the FEM runs in full-PA mode on both boards, which adds ~11 dB
// (Meshtastic measured 17 dBm in -> 28 dBm out), so the SX1262 register
// is set to 11 dBm => ~22 dBm at the antenna on R2 AND R8 alike.
#define TX_POWER_DBM    11        // SX1262 register value
#define EIRP_DBM        22        // radiated power reported on screen/serial
#define PREAMBLE_LEN    8
#define SYNC_WORD       RADIOLIB_SX126X_SYNC_WORD_PRIVATE   // 0x12

// ======================== test parameters =============================
#define PAYLOAD_LEN        32     // section 4: fixed payload length
#define PACKETS_PER_RUN    25     // rounds per run
#define PACKET_INTERVAL_MS 1000   // section 4: 1 packet/s

// ======================= talkback (v2) ===============================
#define TB_WAIT_MS   200          // window to catch the T3S3 talkback packet

// ============================== radio =================================
SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

// packet-ready flag set from the SX1262 DIO1 interrupt
volatile bool receivedFlag = false;
void IRAM_ATTR onPacket(void) { receivedFlag = true; }

char     payload[PAYLOAD_LEN + 1];
char     ackBuf[PAYLOAD_LEN + 1];
uint32_t seq        = 0;
bool     runActive  = false;
uint32_t nextTxMs   = 0;
bool     btnIdle    = true;
uint32_t btnChange  = 0;
uint32_t lastAirMs  = 0;                  // airtime of the last packet
uint32_t tbOk       = 0;                  // talkbacks received
uint32_t tbLost     = 0;                  // talkbacks missed
float    lastTbRssi = 0;                  // our RSSI of the last talkback
float    lastTbSnr  = 0;                  // our SNR of the last talkback
char     oledStatus[28] = "booting";      // line 4 on the screen
bool     oledReady  = false;              // display ACKed on I2C

// ============================== OLED ==================================
// Raw I2C probe first, so a blank screen can be diagnosed over serial.
bool oledProbe(uint8_t *found) {
  *found = 0;
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  delay(10);

  // the panel needs a moment after the reset pulse - retry before giving up
  int lastErr = -1;
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    Wire.beginTransmission(OLED_ADDR);
    lastErr = Wire.endTransmission();
    if (lastErr == 0) { *found = OLED_ADDR; return true; }
    delay(50);
  }

  // full scan - reports whatever is on the bus (e.g. 0x3D instead of 0x3C)
  uint8_t seen[8], n = 0;
  for (uint8_t a = 0x08; a <= 0x77 && n < 8; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      seen[n++] = a;
      if (a == 0x3C || a == 0x3D) *found = a;
    }
  }
  sprint(F("# I2C scan: "));
  if (n == 0) {
    sprint(F("no device ACKed (bus dead / screen unplugged)"));
  } else {
    for (uint8_t i = 0; i < n; i++) {
      if (i) sprint(' ');
      sprint(F("0x"));
      if (seen[i] < 0x10) sprint('0');
      sprint(seen[i], HEX);
    }
  }
  sprint(F("  (probe err="));
  sprint(lastErr);
  sprintln(')');
  return (*found != 0);
}

void oledInit() {
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH); delay(1);
  digitalWrite(OLED_RST, LOW);  delay(20);
  digitalWrite(OLED_RST, HIGH); delay(100);   // SSD1315 settle time after RES#

  uint8_t found = 0;
  oledReady = oledProbe(&found);
  sprint(F("# OLED 0x"));
  sprint(OLED_ADDR, HEX);
  sprintln(oledReady ? F(": found") : F(": NOT FOUND"));

  display.init();
  display.setContrast(255);
  display.flipScreenVertically();

  if (!oledReady) {
    snprintf(oledStatus, sizeof(oledStatus), "OLED NOT FOUND");
    return;
  }

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, F("Heltec V4 TX"));
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 24, String(DEVICE_ID) + "  " + String(FREQ_MHZ, 1) +
                           "MHz SF" + String(SPREAD_FACTOR) +
                           " " + String(EIRP_DBM) + "dBm");
  display.drawString(0, 36, F("booting ..."));
  display.drawString(100, 3, BUILD_MARK);
  display.display();
}

void oledDraw() {
  if (!oledReady) return;

  char     line[44];
  uint32_t sent = (seq > 0) ? seq - 1 : 0;

  display.clear();

  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, DEVICE_ID);

  display.setFont(ArialMT_Plain_10);
  display.drawString(46, 3,
                     runActive ? "RUNNING"
                               : (sent >= PACKETS_PER_RUN ? "COMPLETE" : "STANDBY"));
  display.drawString(100, 3, BUILD_MARK);

  snprintf(line, sizeof(line), "sent %03lu/%d  %lums",
           (unsigned long)sent, PACKETS_PER_RUN, (unsigned long)lastAirMs);
  display.drawString(0, 20, line);

  // talkback tally + our last measurement of the T3S3 talkback
  char tbRes[12];
  if (tbOk + tbLost == 0) {
    snprintf(tbRes, sizeof(tbRes), "--");
  } else {
    snprintf(tbRes, sizeof(tbRes), "%.0f %.2f", lastTbRssi, lastTbSnr);
  }
  snprintf(line, sizeof(line), "TB %03lu/%03lu  %s",
           (unsigned long)tbOk, (unsigned long)(tbOk + tbLost), tbRes);
  display.drawString(0, 32, line);

  if (runActive) {
    display.drawString(0, 44, oledStatus);
  } else {
    String cfg = String(FREQ_MHZ, 1) + "MHz SF" + String(SPREAD_FACTOR) +
                 " " + String(EIRP_DBM) + "dBm";
    display.drawString(0, 44, cfg);
  }

  uint8_t pct = (uint8_t)((sent * 100UL) / PACKETS_PER_RUN);
  display.drawProgressBar(0, 54, 128, 8, pct);

  display.display();
}

void oledMessage(const __FlashStringHelper *l1, const char *l2) {
  if (!oledReady) return;
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, l1);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 26, l2);
  display.display();
}

void printConfig() {
  sprint(F("# dev="));           sprint(DEVICE_ID);
  sprint(F(" freq="));           sprint(FREQ_MHZ, 1);
  sprint(F("MHz bw="));          sprint(BANDWIDTH_KHZ, 0);
  sprint(F("kHz sf="));          sprint(SPREAD_FACTOR);
  sprint(F(" cr=4/"));           sprint(CODING_RATE);
  sprint(F(" power="));          sprint(EIRP_DBM);
  sprint(F("dBm preamble="));    sprint(PREAMBLE_LEN);
  sprint(F(" sync=0x12 payload=")); sprint(PAYLOAD_LEN);
  sprint(F("B packets="));       sprint(PACKETS_PER_RUN);
  sprint(F(" interval="));       sprint(PACKET_INTERVAL_MS);
  sprintln(F("ms"));
}

void printHelp() {
  sprintln(F("# commands: g = start/restart run, h = help"));
}

void startRun() {
  seq       = 1;                     // packets are numbered 000001 .. 000025
  runActive = true;
  nextTxMs  = millis() + 200;
  tbOk      = 0;
  tbLost    = 0;
  snprintf(oledStatus, sizeof(oledStatus), "run start");
  sprintln();
  sprintln(F("# RUN START"));
  printConfig();
  printHelp();
  oledDraw();
}

void printSeq() {
  char b[8];
  snprintf(b, sizeof(b), "%06lu", (unsigned long)seq);
  sprint(b);
}

bool sendPacket() {
  // fixed 32 byte payload: "<id>,<seq 6 digits>" padded with '.'
  snprintf(payload, sizeof(payload), "%s,%06lu",
           DEVICE_ID, (unsigned long)seq);
  size_t n = strlen(payload);
  for (size_t i = n; i < PAYLOAD_LEN; i++) payload[i] = '.';
  payload[PAYLOAD_LEN] = '\0';

  uint32_t t0 = millis();
  int state = radio.transmit((uint8_t *)payload, PAYLOAD_LEN);
  uint32_t took = millis() - t0;
  lastAirMs = took;

  if (state == RADIOLIB_ERR_NONE) {
    snprintf(oledStatus, sizeof(oledStatus), "TX ok");
    // transmitter,packet number,airtime ms
    sprint(F("TX,"));       sprint(DEVICE_ID);
    sprint(F(","));         printSeq();
    sprint(F(","));         sprintln(took);
    return true;
  }

  snprintf(oledStatus, sizeof(oledStatus), "TX ERR %d", state);
  sprint(F("# TX ERROR seq="));
  sprint(seq);
  sprint(F(" code="));
  sprintln(state);
  return false;
}

// wait up to TB_WAIT_MS for the T3S3 talkback "T3,<seq>";
// measures the talkback packet's RSSI / SNR ourselves
bool waitTalkback(uint32_t expectSeq, float *rssi, float *snr) {
  receivedFlag = false;                       // drop any TX_DONE edge
  int ar = radio.startReceive();
  if (ar != RADIOLIB_ERR_NONE) return false;

  uint32_t t0 = millis();
  while (!receivedFlag && (int32_t)(millis() - t0) < TB_WAIT_MS) delay(1);
  if (!receivedFlag) return false;
  receivedFlag = false;

  size_t len = radio.getPacketLength();
  if (len == 0 || len > 63) return false;
  char tb[64];
  if (radio.readData((uint8_t *)tb, len) != RADIOLIB_ERR_NONE) return false;
  *rssi = radio.getRSSI();                    // of THIS talkback packet
  *snr  = radio.getSNR();
  tb[len] = '\0';

  char *comma = strchr(tb, ',');
  if (!comma || strncmp(tb, "T3,", 3) != 0) return false;
  return strtoul(comma + 1, NULL, 10) == expectSeq;
}

// reply to the T3S3 with OUR measured RSSI / SNR of its talkback
void sendAck(uint32_t ackSeq, float rssi, float snr) {
  snprintf(ackBuf, sizeof(ackBuf), "AK,%06lu,%.0f,%.2f",
           (unsigned long)ackSeq, rssi, snr);
  size_t n = strlen(ackBuf);
  for (size_t i = n; i < PAYLOAD_LEN; i++) ackBuf[i] = '.';
  ackBuf[PAYLOAD_LEN] = '\0';
  radio.transmit((uint8_t *)ackBuf, PAYLOAD_LEN);
  receivedFlag = false;                       // drop TX_DONE edge
}

void handleTalkback() {
  snprintf(oledStatus, sizeof(oledStatus), "wait TB");
  float rssi = 0, snr = 0;
  if (waitTalkback(seq, &rssi, &snr)) {
    lastTbRssi = rssi;
    lastTbSnr  = snr;
    tbOk++;
    snprintf(oledStatus, sizeof(oledStatus), "TB ok %.0fdBm", rssi);
    sprint(F("TB ok seq="));  printSeq();
    sprint(F(" rssi="));      sprint(rssi);
    sprint(F(" snr="));       sprintln(snr);
    sendAck(seq, rssi, snr);
  } else {
    tbLost++;
    snprintf(oledStatus, sizeof(oledStatus), "TB LOST");
    sprint(F("# TB LOST seq="));
    sprintln(seq);
  }
}

void handleSerial() {
#if SERIAL_DBG
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' || c == 'G') {
      startRun();
    } else if (c == 'h' || c == 'H' || c == '?') {
      printHelp();
    }
  }
#endif
}

void handleBootButton() {
  bool pressed = (digitalRead(BOOT_BTN) == LOW);
  if (pressed == btnIdle) return;                 // no edge
  if (millis() - btnChange < 50) return;          // debounce
  btnChange = millis();
  btnIdle   = !pressed;
  if (pressed && !runActive) startRun();          // start when idle only
}

void halt(const __FlashStringHelper *msg) {
  sprint(F("# FATAL: "));
  sprintln(msg);
  if (oledReady) {
    char buf[40];
    snprintf(buf, sizeof(buf), "%s", String(msg).c_str());
    oledMessage(F("FATAL"), buf);
  }
  while (true) delay(10);
}

void setup() {
#if SERIAL_DBG
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
#endif

  // KCT8103L front end: LDO first, then enable, then PA mode - kept on
  // for the whole session (floating mode pins = undefined output power)
  pinMode(VFEM_LDO, OUTPUT);
  digitalWrite(VFEM_LDO, HIGH);
  pinMode(FEM_EN, OUTPUT);
  digitalWrite(FEM_EN, HIGH);
  pinMode(FEM_CTX, OUTPUT);
  digitalWrite(FEM_CTX, HIGH);
  delay(10);

  pinMode(BOOT_BTN, INPUT_PULLUP);

  // Vext rail (active LOW): powers the OLED and the LoRa antenna boost
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  delay(50);

  // screen first - so a blank display or a radio failure is visible
  oledInit();

  // dedicated SPI bus for the SX1262 (pins are not the variant defaults)
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  sprint(F("# SX1262 init ... "));
  int state = radio.begin(FREQ_MHZ, BANDWIDTH_KHZ, SPREAD_FACTOR, CODING_RATE,
                          SYNC_WORD, TX_POWER_DBM, PREAMBLE_LEN);
  if (state != RADIOLIB_ERR_NONE) {
    sprint(F("failed, code "));
    sprintln(state);
    snprintf(oledStatus, sizeof(oledStatus), "RADIO ERR %d", state);
    halt(F("radio.begin() - check wiring / board settings"));
  }
  sprintln(F("ok"));

  // explicit header, CRC is already enabled by radio.begin()
  radio.explicitHeader();

  // DIO1 interrupt -> talkback wait loop
  radio.setPacketReceivedAction(onPacket);

  snprintf(oledStatus, sizeof(oledStatus), "radio ok");
  oledDraw();

  sprintln(F("# Heltec V4 TX ready"));

  // if loop() ever hangs (I2C/SPI/CDC), the task watchdog panics after
  // ~5 s with a backtrace on serial instead of freezing silently
  enableLoopWDT();

  startRun();
}

void loop() {
  handleSerial();
  handleBootButton();

  if (runActive && (int32_t)(millis() - nextTxMs) >= 0) {
    if (sendPacket()) {
      handleTalkback();
    }
    seq++;
    oledDraw();

    // seq is incremented after the send, so 101 means packet 100 is done
    if (seq > PACKETS_PER_RUN) {
      runActive = false;
      radio.standby();
      oledDraw();
      sprint(F("# RUN COMPLETE - "));
      sprint(PACKETS_PER_RUN);
      sprintln(F(" packets sent, radio in standby"));
      sprintln(F("# send 'g' or press BOOT to start the next run"));
    } else {
      nextTxMs += PACKET_INTERVAL_MS;
      // if we fell behind, resync instead of bursting
      if ((int32_t)(millis() - nextTxMs) >= 0) nextTxMs = millis() + PACKET_INTERVAL_MS;
    }
  }
}
