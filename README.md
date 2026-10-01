# Heltec V4 SAW Test

Firmware for comparing the RF performance of two **Heltec WiFi LoRa 32 V4**
boards — an **R2** (ESP32-S3R2) and an **R8** (ESP32-S3R8) — using a fixed
**LilyGO T3-S3** reference receiver that logs RSSI/SNR for every packet.

All three devices run the same radio configuration; each transmitter sends
exactly **100 packets at 1 packet/second** with a fixed 32-byte payload, and
the receiver writes one CSV line per packet.

## Repository layout

| Sketch | Device | Role |
|---|---|---|
| [`R2/R2.ino`](R2/R2.ino) | Heltec V4, ESP32-S3R2 | transmitter (`DEVICE_ID "R2"`) |
| [`R8/R8.ino`](R8/R8.ino) | Heltec V4, ESP32-S3R8 | transmitter (`DEVICE_ID "R8"`) |
| [`T3S3_RX/T3S3_RX.ino`](T3S3_RX/T3S3_RX.ino) | LilyGO T3-S3 (SX1262) | reference receiver / CSV logger |

## Requirements

- Arduino IDE
- **esp32 Arduino core** (tested with 3.3.x)
- Libraries (via Library Manager):
  - [RadioLib](https://github.com/jgromes/RadioLib) (tested with 7.4.0)
  - [ESP8266 and ESP32 OLED driver for SSD1306 displays](https://github.com/ThingPulse/esp8266-oled-ssd1306) (ThingPulse)
- Antennas for 915–928 MHz on all three boards

No Heltec board package is required — the sketches define their own pins.

## Board settings

### Heltec V4 transmitters (`R2`, `R8`)

| Option | Value |
|---|---|
| Board | `ESP32S3 Dev Module` |
| USB CDC On Boot | Enabled |
| Flash Size | 16MB (128Mb) |
| PSRAM | **Disabled** (see note) |
| Partition Scheme | Default 4MB with spiffs |
| Upload Speed | 921600 |

> **PSRAM must be Disabled.** On the R2, quad PSRAM owns GPIO33–37 including
> **GPIO36 = Vext**, the OLED + antenna-boost power rail. Enabling PSRAM
> steals the pin → dark screen and dead antenna boost.

### LilyGO T3-S3 receiver

| Option | Value |
|---|---|
| Board | `LilyGo T3-S3` |
| Revision | `Radio_SX1262` |
| USB CDC On Boot | Enabled |
| PSRAM | Enabled (QSPI) |
| Upload Speed | 921600 |

## Flashing

1. Open the sketch for the target device and upload it.
2. The transmitters show a build mark **`w3`** in the top-right corner of the
   OLED — if it is missing or outdated, the wrong build is on the board.

## Running a test

1. Place the receiver where it can hear both transmitters (or move it to a
   marked distance — see step 3).
2. Power the transmitter: a run **starts automatically at boot** (100 packets,
   1/s). Press the **BOOT** button (GPIO0) to start another run when idle.
3. On the receiver's serial console (115200 baud) set the distance label:

   | Command | Effect |
   |---|---|
   | `d<number>` | set test distance, e.g. `d10` → field `010m` |
   | `r` | reset statistics (start of a new run) |
   | `s` | print statistics summary |
   | `h` / `?` | help |

4. CSV lines are printed live:

   ```
   R2,010m,000001,-71,8.25
   <device>,<distance>,<sequence>,<RSSI dBm>,<SNR dB>
   ```

   Lines starting with `#` are comments/summaries, not data.

### Transmitter OLED

Device ID, RUNNING/STANDBY/COMPLETE state, packets sent, packet airtime,
radio config, status line and a progress bar. After packet 100 the radio
goes to standby and shows `COMPLETE`.

## Radio configuration (all three devices)

| Parameter | Value |
|---|---|
| Frequency | 923.2 MHz (AS923, Singapore 915–928 MHz) |
| Bandwidth | 125 kHz |
| Spreading factor | SF7 |
| Coding rate | 4/5 |
| Preamble | 8 symbols |
| Sync word | 0x12 (private) |
| Payload | fixed 32 bytes: `R2,000001...........` / `R8,000001...` |
| Packet rate | 1 packet/s, exactly 100 packets per run |
| TX power | SX1262 register **11 dBm** → **~22 dBm radiated** |

## Transmitter front-end (FEM)

Both test boards are **V4.3** hardware with a KCT8103L front-end module.
The firmware drives it explicitly at boot:

| Pin | Level | Function |
|---|---|---|
| GPIO7 | HIGH | FEM LDO power |
| GPIO2 | HIGH | FEM enable (CSD) |
| GPIO5 | HIGH | TX PA mode (CTX; CPS = SX1262 DIO2, automatic) |
| GPIO36 (R2) / GPIO40 (R8) | LOW | Vext: OLED + antenna boost (active LOW) |

Radiated power ≈ 11 dBm register + ~11 dB FEM gain ≈ **22 dBm** (nominal,
±2 dB), identical on both boards so R2-vs-R8 comparisons stay valid.

## Notes

- The transmitters perform **no serial I/O** (`SERIAL_DBG 0`): USB CDC
  activity froze them mid-run when the serial monitor was closed. Flip the
  flag in the sketch header to 1 to re-enable the `TX,...` log lines and the
  `g`/`h` commands.
- A task watchdog (`enableLoopWDT()`) is armed on the transmitters as a
  safety net: a hung loop panics and reboots after 5 s instead of freezing
  silently.
- The receiver keeps its serial console — that is how the CSV is captured.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Dark OLED on the R2 | PSRAM not set to Disabled (GPIO36 stolen) |
| Dim OLED | Vext not driven (R2: GPIO36, R8: GPIO40) |
| RSSI ≈ −65…−78 dBm at point-blank | FEM LDO (GPIO7) not enabled or wrong FEM mode pin |
| Transmitter freezes mid-run | Serial I/O was enabled; this build has `SERIAL_DBG 0` |
| OLED doesn't show `w3` | Wrong or outdated firmware on the board |
| R2 and R8 differ widely at the same spot | Check both run matching `w3` builds and identical antenna placement |
