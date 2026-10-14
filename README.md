# the-receiver

433 MHz weather station receiver for **Geevon TX19** outdoor sensor.

Decodes temperature, humidity and channel from a Geevon TX19 (FCC ID: 2AM88-TX19)
transmitter using an **RX470C-V01** superheterodyne receiver module and a
**NodeMCU D1 Mini (ESP-12F)**.

## Hardware

| Component            | Role                    |
|----------------------|-------------------------|
| NodeMCU D1 Mini      | Microcontroller (ESP8266, 80 MHz) |
| RX470C-V01           | 433 MHz ASK/OOK receiver |
| Geevon TX19 sensor   | Outdoor temperature/humidity transmitter |

### Wiring

| RX470C-V01 | NodeMCU D1 Mini |
|------------|-----------------|
| VCC        | VIN (5V)        |
| GND        | GND             |
| D0         | D2 (GPIO4)      |

**Important:** The receiver outputs 5V logic. A voltage divider (10 kΩ + 22 kΩ)
is required between D0 and D2 to bring the signal within the ESP8266's 3.3V
tolerance (resulting in ~3.44V at idle, safely under the 3.6V absolute maximum).

```
RX470 D0 ──┬── 10 kΩ ──┬── D2 (GPIO4)
           │           │
           │          22 kΩ
           │           │
          GND         GND
```

### Antenna

A **17.3 cm** wire (quarter-wave for 433 MHz) soldered to the ANT pad on the
RX470C-V01 module.

## Protocol

Reverse-engineered from live captures. The sensor (Geevon TX19 **no LCD**) uses
**OOK space-encoded** modulation with the following measured timings:

| Symbol       | Duration       | Meaning |
|--------------|---------------|---------|
| Mark (carrier) | 300–1000 µs  | Fixed pulse width (sync) |
| Short space  | 700–1100 µs   | Bit 0   |
| Long space   | 1500–5000 µs  | Bit 1   |

**Note:** This is NOT the 72-bit TX19-1 (with LCD) protocol. Do not use
`rtl_433/src/devices/geevon_tx19.c` constants.

### Packet format (40 bits / 5 bytes)

```
Byte 0    Byte 1    Byte 2    Byte 3    Byte 4
IIIIIIII  IICC?HHH  TTTTTTTT  TTTThhhh  hhhhhhhh
```

| Field       | Position     | Size | Description |
|-------------|-------------|------|-------------|
| ID          | d[0]–d[1]   | 16   | Sensor ID; only `id & 0x1FC0` is stable (top 3 bits of d[0] drift: 1D99, 3D99 … FD99 are the same sensor) |
| Channel     | d[1] >> 4   | 4    | `8→CH1`, `9→CH2`, `A→CH3` |
| Battery     | d[1] & 0x08 | 1    | 0 = OK                     |
| Temperature | d[2]..d[3]  | 12   | `(d[2]<<4 \| d[3]>>4) × 0.1 °C` |
| Humidity    | d[4]         | 8    | Relative humidity %        |

**Decoding:**
```c
channel = (d[1] >> 4) - 7;                       // CH1/CH2/CH3
tempC   = (((int)d[2] << 4) | (d[3] >> 4)) * 0.1f;  // 12-bit, NO -500 offset
hum     = d[4];                                       // byte 4 only
```

**Scanner behaviour:**
- Iterates all bit offsets and both inversions (inv=0, inv=1)
- Filters for CH2 only
- Uses 12-bit temperature formula (no alternative)
- Tracks the sensor ID (`lastGoodId`) and offset (`lastGoodOff`) to stay locked
  on the correct packet across transmissions
- **Hard sensor ID filter:** if `sensorId` is configured (non-zero in EEPROM),
  only packets whose `id & 0x1FC0` equals it are accepted — all others are silently discarded.
  The value is masked with `0x1FC0` when saved and when loaded, so `9D99` and `1D80` are equivalent.
- **Tail check:** `d[3] & 0x0F` must be `0xF` (true for every good packet in a 41 h capture).
- **Humidity smoothing:** Rejects the entire packet if humidity jumps greater than ±5% unless more than
  10 minutes (`STALE_TIMEOUT`) have passed since the last valid transmission,
  preventing random humidity noise or wrong-sensor hijack.

**Temperature formula confirmed:**
- Raw 12-bit = `(d[2] << 4) | (d[3] >> 4)` (e.g. `0x118` = 280 → 28.0 °C)
- NO -500 offset (TX19-1 uses offset, this device does not)

**Humidity confirmed:** `d[4]` only (not split across d[3]/d[4]).

> **Station display range:** The base station's spec states *"The indoor and outdoor
> humidity range is 20%–95%"*. Values below 20% are displayed as 20%; values above
> 95% are displayed as 95%. Our decoder reports the real sensor value from `d[4]`
> without clipping.

## Algorithm

1. **GPIO interrupt** captures every pin state change (CHANGE mode) and records
   the microsecond timestamp. A gap >5 ms (`IDLE_TIMEOUT`) resets the pulse
   buffer, treating the next pulses as a new packet.

2. **Bit extraction** — marks (carrier) and spaces are paired:
   - `mark` 300–1000 µs + `space` 700–1100 µs → **bit 0**
   - `mark` 300–1000 µs + `space` 1500–5000 µs → **bit 1**
   - Anything else → invalid

3. **40-bit scan** — every possible offset and inversion is tested. Candidates
   are validated by channel nibble (must be 8/9/A) and temperature range.

4. **Best-match** — among all CH2 candidates, the one with temperature closest
   to 29 °C is selected, with bonuses for matching last known sensor ID and
   bit offset (prevents drift to neighbour sensors on the same channel).
   When a hard `sensorId` is configured, only matching candidates are considered.

## Usage

### Flashing

1. Open `the-receiver.ino` in Arduino IDE
2. Select board: **NodeMCU 1.0 (ESP-12E Module)**
3. Install libraries: **PubSubClient** by Nick O'Leary
4. Upload to the controller

### First Boot — Configuration

On first boot (or after EEPROM reset) the ESP starts an access point:

1. Connect your phone/PC to WiFi SSID **the-receiver**
2. Open http://192.168.4.1
3. Enter your WiFi credentials and MQTT broker details
4. Optionally, set **Sensor ID** (hex) to lock the receiver to a specific
   sensor — use the `stable=0xXXXX` value from the serial log line `RF: CH2 id=0x... (stable=0x...)`.
   The active filter is printed at boot as `Sensor ID filter: 0x...`. Leave `0` for auto.
5. Optionally, set the **NTP server** (empty = `pl.pool.ntp.org`) and the **GMT offset** in hours
   (e.g. `1`, `-5`, `5.5`; empty = Warsaw time, CET/CEST with automatic daylight saving).
   A fixed GMT offset has no daylight saving.
6. Click "Save & Reboot"

The device will then connect to your WiFi network, sync the clock over NTP and connect to the MQTT broker.

Startup order: WiFi → NTP sync (up to 15 s) → `Sensor ID filter` line → MQTT → RF decoding starts.
The clock is re-synced every 3 hours. Log timestamps `[hh:mm:ss.ms]` show local (Warsaw by default)
time once NTP is synced; until then (or if NTP fails) they show uptime.

### Home Assistant

If MQTT Discovery is enabled in Home Assistant, two sensors
(temperature and humidity) appear automatically under device **the-receiver**.

**State topic:** `the-receiver/state` (JSON: `{"temperature":24.4,"humidity":57}`)  
**Availability topic:** `the-receiver/availability` (`online` / `offline`, retained)

### Re-configuration

WiFi fail 10 times in a row → boots into AP config mode automatically.
To force re-configuration, clear EEPROM (e.g. by erasing flash via `esptool.py
erase_flash` or uploading an empty sketch).

### Serial Monitor

Output at 115200 baud (USB-only; silent when no serial host):

```
=== the-receiver ===
[00:00:00.120] Config found in EEPROM
[00:00:00.125] Connecting WiFi to MyNetwork
[00:00:04.406] WiFi OK: 192.168.1.42
[00:00:04.410] NTP: syncing with pl.pool.ntp.org
[16:04:05.512] NTP: synced 2026-10-14 16:04:05 (UTC+2:00, Warsaw)
[15:04:05.514] Sensor ID filter: 0x1D80
[15:04:05.620] MQTT OK
```

Received RF packets are logged with sensor ID:
```
[14:23:45.123] RF: CH2 id=0x9D99 (stable=0x1D80)  24.5C  57%  raw:9D 99 F0 1F 39
```

The built-in blue LED blinks briefly on each valid RF packet.

### Capturing the log (`capture_log.py`)

`capture_log.py` records the serial output to a file, e.g. for long test runs
on a Windows PC.

```
pip install pyserial
python capture_log.py COM3
```

| Option   | Default      | Description |
|----------|--------------|-------------|
| `port`   | (required)   | Serial port name, e.g. `COM3` |
| `--baud` | `115200`     | Baud rate |
| `--out`  | `output.log` | Output file, relative to the current directory |

- Lines are written as received (the firmware already adds `[hh:mm:ss.ms]`),
  flushed immediately and echoed to the console.
- The file is opened in append mode, so existing logs are kept.
- If the port disappears (USB unplugged), the script retries every 2 s and
  keeps appending to the same file. Stop with Ctrl+C.
- Opening the port usually resets the D1 Mini via DTR, so the log starts with
  the boot messages (`Sensor ID filter: ...`).
- `output.log` is listed in `.gitignore`.

## References

- [rtl_433](https://github.com/merbanan/rtl_433) — Geevon TX19 decoder (issue #3438)
