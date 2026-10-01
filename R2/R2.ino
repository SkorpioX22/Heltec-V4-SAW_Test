/*
  Heltec WiFi LoRa 32 V4 (ESP32-S3R2, board "R2") - RSSI/SNR test transmitter
  Part of: Heltec V4 RSSI / SNR Test Procedure (T3S3 reference receiver)
  NOTE: the R8 board uses the separate sketch ..\R8\R8.ino

  Role: transmits numbered, fixed-length 32 byte packets at 1 packet/s.
        Flash THIS sketch to the R2 board only (DEVICE_ID "R2").
        The R8 board uses ..\R8\R8.ino (DEVICE_ID "R8").
  Radio: SX1262 controlled with RadioLib, AS923 / Singapore (915-928 MHz).

  Arduino IDE settings (esp32 core 3.3.2, no Heltec board package required):
    Board           : "ESP32S3 Dev Module"
                      (or Heltec's "WiFi LoRa 32(V4)" if that package is installed,
                       or "Heltec WiFi LoRa 32(V3)" - the V4 is pin compatible)
    USB CDC On Boot : Enabled
    Flash Size      : 16MB (128Mb)   - V4 has 16 MB external flash
    PSRAM           : Disabled       - REQUIRED on the R2: quad PSRAM owns
                                       GPIO33-37, incl. GPIO36 = Vext (OLED power)
    Partition Scheme: Default 4MB with spiffs
    Upload Speed    : 921600

  Payload (exactly 32 bytes): "R2,000001..........."  (padded with '.')
  Serial log: disabled by default (SERIAL_DBG 0) - USB CDC activity with the
              monitor closed froze the transmitter mid-run. Status is on the
              OLED; CSV logging is done by the T3S3 receiver.
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
#define DEVICE_ID "R2"
#define BUILD_MARK "w3"   // shown top-right on the OLED = this build is running
#define BOARD_R8  0      // chip: 0 = ESP32-S3R2 (Vext=GPIO36), 1 = R8 (Vext=GPIO40)

// Front end chip depends on the BOARD REVISION, not the ESP32 chip:
//   V4.2 -> GC1109  (PA mode = CPS on GPIO46)
//   V4.3 -> KCT8103L (PA mode = CTX on GPIO5)   <-- this board (silkscreen V4.3)
#define FEM_IS_KCT8103L 1

// ===================== Heltec WiFi LoRa 32 V4 pins =====================
// ESP32-S3 + SX1262 + KCT8103L front end (V4.3), pin compatible with the V3.
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_RST   12
#define LORA_BUSY  13
#define LORA_SCK   9
#define LORA_MISO  11
#define LORA_MOSI  10
#define FEM_EN     2      // front end CSD - must stay HIGH
#define VFEM_LDO   7      // FEM LDO power enable (Meshtastic: LORA_PA_POWER 7).
                          // Without this the FEM has no supply and the board
                          // radiates only leakage (~ -78 dBm at point-blank)
#if FEM_IS_KCT8103L
#define FEM_CTX    5      // KCT8103L CTX: HIGH = transmit PA mode (CPS = DIO2)
#else
#define FEM_PA     46     // GC1109 CPS: HIGH = full PA, LOW = bypass
                          // (GPIO35 is the R2 LED - not used here)
#endif
#define BOOT_BTN   0      // push to (re)start a run when idle

// ==================== OLED (Heltec V4 0.96" SSD1315) ==================
// V4 wiring: SDA = GPIO17, SCL = GPIO18, RST = GPIO21 (same on R2 and R8)
#define OLED_SDA   17
#define OLED_SCL   18
#define OLED_RST   21
#define OLED_ADDR  0x3C   // 0x3D on some panels

// OLED power rail (Vext), active LOW:
//   V4 / ESP32-S3R2 ("R2"): GPIO36
//   V4 R8 / ESP32-S3R8    : GPIO40
// Note: on the R2, GPIO36 is also a quad-PSRAM pin (ESP32-S3 PSRAM uses
// GPIO33-37), so Tools -> PSRAM must be "Disabled" or the PSRAM peripheral
// steals this pin and the display loses power.
#if BOARD_R8
#define VEXT_PIN   40
#else
#define VEXT_PIN   36
#endif

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
#define PACKETS_PER_RUN    100    // section 8: exactly 100 packets
#define PACKET_INTERVAL_MS 1000   // section 4: 1 packet/s

// ============================== radio =================================
SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

char     payload[PAYLOAD_LEN + 1];
uint32_t seq        = 0;
bool     runActive  = false;
uint32_t nextTxMs   = 0;
bool     btnIdle    = true;
uint32_t btnChange  = 0;
uint32_t lastAirMs  = 0;                  // airtime of the last packet
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

  String cfg = String(FREQ_MHZ, 1) + "MHz SF" + String(SPREAD_FACTOR) +
               " " + String(EIRP_DBM) + "dBm";
  display.drawString(0, 32, cfg);

  display.drawString(0, 44, oledStatus);

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
  seq       = 1;                     // packets are numbered 000001 .. 000100
  runActive = true;
  nextTxMs  = millis() + 200;
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

  // front end: LDO first, then enable, then PA mode - kept on for the
  // whole session (floating mode pins = undefined output power)
  pinMode(VFEM_LDO, OUTPUT);
  digitalWrite(VFEM_LDO, HIGH);
  pinMode(FEM_EN, OUTPUT);
  digitalWrite(FEM_EN, HIGH);
#if FEM_IS_KCT8103L
  pinMode(FEM_CTX, OUTPUT);
  digitalWrite(FEM_CTX, HIGH);
#else
  pinMode(FEM_PA, OUTPUT);
  digitalWrite(FEM_PA, HIGH);        // full PA (~22 dBm radiated)
#endif
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
    sendPacket();
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
