# Heltec-V4-SAW_Test

Transmitter firmware for comparing the RF performance (RSSI/SNR) of two
Heltec WiFi LoRa 32 **V4** boards — an **R2** (ESP32-S3R2) and an **R8**
(ESP32-S3R8) — against a fixed LilyGO T3S3 reference receiver.

## Layout

| Folder | Sketch | Board | DEVICE_ID |
|---|---|---|---|
| `R2/` | `R2.ino` | Heltec V4, ESP32-S3R2 | `R2` |
| `R8/` | `R8.ino` | Heltec V4, ESP32-S3R8 | `R8` |

The receiver/logger (`t3s3_rx_logger`) lives in the original project:
`C:\Users\dcuyl\Documents\opencode projects\HeltecV4_T3S3_RSSI_SNR_Test\`

## Software setup

- Arduino IDE with **esp32 core** (tested: 3.3.x)
- Libraries (Library Manager):
  - **RadioLib** (tested 7.4.0)
  - **ESP8266 and ESP32 OLED driver for SSD1306 displays** (ThingPulse)

### Board settings (both boards, Arduino IDE Tools menu)

| Option | Value |
|---|---|
| Board | `ESP32S3 Dev Module` |
| USB CDC On Boot | Enabled |
| Flash Size | 16MB (128Mb) |
| PSRAM | **Disabled** (mandatory — see below) |
| Partition Scheme | Default 4MB with spiffs |
| Upload Speed | 921600 |

**PSRAM must be Disabled.** The R2's quad PSRAM owns GPIO33–37, including
**GPIO36 = Vext** (OLED + antenna-boost power rail). With PSRAM enabled the
peripheral steals the pin: dark screen and dead antenna boost.

## Flashing

1. Connect the board by USB.
2. Open `R2/R2.ino` → upload to the **R2** board; open `R8/R8.ino` → upload
   to the **R8** board.
3. Confirm the build mark **`w3`** in the top-right corner of the OLED. No
   mark (or an old mark) = wrong build on the board.

No serial monitor is used: `SERIAL_DBG` is `0` — the sketches do no serial
I/O at all (serial activity with the monitor closed froze the transmitter
mid-run; the watchdog `enableLoopWDT()` stays armed as a safety net).

## Operation

- The run **starts automatically** at boot: 100 packets, 1 packet/second.
- **BOOT button** (GPIO0): starts a new run when idle.
- OLED: device ID, RUNNING/STANDBY/COMPLETE, packets sent, airtime,
  radio config (`923.2MHz SF7 22dBm`), status line, progress bar.
- At packet 100 the radio goes to standby (`COMPLETE`); reset or press
  BOOT to run again.

## Radio configuration (identical on R2, R8 and the receiver)

| Parameter | Value |
|---|---|
| Frequency | 923.2 MHz (AS923, Singapore 915–928 MHz) |
| Bandwidth | 125 kHz |
| Spreading factor | SF7 |
| Coding rate | 4/5 |
| Preamble | 8 symbols |
| Sync word | 0x12 (private) |
| Payload | fixed 32 bytes: `R2,000001...........` / `R8,000001...` |
| Packet rate | 1/s, exactly 100 packets per run |
| TX power | SX1262 register **11 dBm** → **~22 dBm radiated** |

### Front-end (FEM) notes

Both test boards are **V4.3** (KCT8103L FEM), driven explicitly at boot:

- `GPIO7` HIGH — FEM LDO power (without it: ~−78 dBm at point-blank)
- `GPIO2` HIGH — FEM chip enable (CSD)
- `GPIO5` HIGH — TX PA mode (CTX; CPS = SX1262 DIO2, automatic)
- Vext (antenna boost + OLED), active LOW: **GPIO36 on R2, GPIO40 on R8**

Radiated power ≈ 11 dBm + ~11 dB FEM gain ≈ **22 dBm** (nominal, ±2 dB,
equal on both boards so R2-vs-R8 comparisons stay valid). The procedure
originally specified 17 dBm; it was amended — see the original project's
`PROCEDURE.md`.

## Receiver / CSV

The T3S3 logs one CSV line per packet:

```
R2,010m,000001,-71,8.25
<device>,<distance>,<sequence>,<RSSI dBm>,<SNR dB>
```

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Dark OLED on R2 | PSRAM not Disabled (GPIO36 stolen) |
| Dim OLED | Vext not driven (R2: GPIO36, R8: GPIO40) |
| RSSI ≈ −65…−78 dBm point-blank | FEM LDO (GPIO7) not enabled, or wrong FEM mode pin |
| Transmitter freezes mid-run | Was serial I/O; this build has `SERIAL_DBG 0` |
| OLED doesn't show `w3` | Wrong/old firmware on the board |
| R2 and R8 differ widely at same distance | Check both run matching `w3` builds and same antenna placement |
