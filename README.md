# the-receiver — Geevon TX19 433 MHz sensor receiver for ESP8266 (MQTT / Home Assistant)

DIY **ESP8266 (NodeMCU D1 Mini) receiver for the Geevon TX19 433 MHz outdoor
temperature and humidity sensor**. Decodes the sensor's OOK radio packets with an
**RX470C-V01** receiver module and publishes them over **MQTT** with automatic
**Home Assistant** discovery. No SDR dongle or `rtl_433` needed.

Target transmitter: Geevon TX19 (FCC ID: 2AM88-TX19), receiver module RX470C-V01
(superheterodyne), board NodeMCU D1 Mini (ESP-12F). Decoded: temperature, humidity, channel.

Features: 40-bit TX19 (no LCD) packet decoder, sensor ID lock, WiFi/MQTT/NTP setup
through a web page, noise and humidity-jump filtering, Arduino IDE sketch.

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
tolerance (resulting in ~3.44V at logic high, safely under the 3.6V absolute maximum).

```
RX470 D0 ──── 10 kΩ ──┬── D2 (GPIO4)
                      │
                     22 kΩ
                      │
                     GND
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
hhhIIIII  CCCCffff  TTTTTTTT  TTTT1111  HHHHHHHH
```

| Field       | Position       | Size | Description |
|-------------|----------------|------|-------------|
| `hhh`       | d[0] >> 5      | 3    | Copy of the 3 low bits of humidity (`d[4] & 7`), see *Integrity check* below. **Not** part of the ID |
| ID          | d[0] & 0x1F    | 5    | Sensor ID (`0x1D` for the author's sensor). The firmware compares `id & 0x1FC0`, which adds the two constant top bits of the channel nibble |
| Channel     | (d[1] >> 4) & 3 | 2   | `0→CH1`, `1→CH2`, `2→CH3` (same bits as rtl_433 TX19-1; the decoder accepts CH2 only) |
| Bit 7       | d[1] >> 7      | 1    | Always 1 in all captured packets. In rtl_433 TX19-1 this is the battery bit (1 = low); polarity for this sensor unconfirmed |
| Bit 6       | d[1] >> 6 & 1  | 1    | Always 0 in all captured packets |
| Flags       | d[1] & 0x0F    | 4    | Observed `0x9` in 99.9 % of packets, `0x1` in 2 of 1796; meaning unknown, the decoder ignores it |
| Temperature | d[2]..d[3]     | 12   | `(d[2]<<4 \| d[3]>>4) × 0.1 °C` |
| Constant    | d[3] & 0x0F    | 4    | Always `0xF` in good packets (used as a tail check) |
| Humidity    | d[4]           | 8    | Relative humidity %        |

#### Integrity check: `d[0] >> 5 == d[4] & 7`

The 40-bit packet has no checksum, but its first 3 bits repeat the last 3 bits of the
humidity byte: `(d[0] >> 5) == (d[4] & 7)`. Example: humidity 77 % = `0x4D`, low bits `101`
→ `d[0] = 0xBD` (`101 11101`); 78 % → `0xDD`, 79 % → `0xFD`.

Why this was found: the first byte of the same sensor looked like it changed all the time
(`1D 3D 5D 7D 9D BD DD FD` followed by `99`). Only the low 5 bits (`0x1D`) are the ID; the top
3 bits follow the humidity, changing exactly when the humidity changes.

Evidence from two captures (41 h and 2.2 h, 1872 packets): the rule holds in 1819 packets
(97.2 %); random bits would match in 12.5 %. The ~3 % that fail are corrupted packets:
14 of the 18 packets stopped by the humidity-jump filter fail it, and 35 packets that were
accepted also failed it, so their humidity could be off by up to 7 %. Packets in a real
weather change (a storm: humidity 71 → 80 %) satisfy the rule.

**Status:** enforced by the firmware: packets that fail it are discarded. This costs about
3-5 % of packets (the sensor transmits every ~67 s, and about 17 % are already lost to
reception) and keeps corrupted humidity values out of MQTT. Example caught by it: `BD 99 06 6F 5B`
(91 % instead of the real 86 %, jump of exactly 5 so the jump filter let it through).

**Decoding:**
```c
channel = ((d[1] >> 4) & 3) + 1;                 // CH1/CH2/CH3
tempC   = (((int)d[2] << 4) | (d[3] >> 4)) * 0.1f;  // 12-bit, NO -500 offset
hum     = d[4];                                       // byte 4 only
```

**Scanner behaviour:**
- Iterates all bit offsets and both inversions (inv=0, inv=1)
- Accepts only the configured channel (CH1–CH3, default CH2); accepts temperature -30…60 °C and humidity ≤ 100 %
- Uses 12-bit temperature formula (no alternative)
- Tracks the sensor ID (`lastGoodId`) and offset (`lastGoodOff`) to stay locked
  on the correct packet across transmissions
- **Hard sensor ID filter:** if `sensorId` is configured (non-zero in EEPROM),
  only packets whose `id & 0x1FC0` equals it are accepted — all others are silently discarded.
  The value is masked with `0x1FC0` when saved and when loaded, so `9D99` and `1D80` are equivalent.
- **Tail check:** `d[3] & 0x0F` must be `0xF` (true for every good packet in a 41 h capture).
- **Integrity check:** `d[0] >> 5` must equal `d[4] & 7` (see *Integrity check* above).
- **Humidity smoothing:** a packet whose humidity jumps by more than ±5% is held back
  (`--- hum jump pending`) and not published. If the next packet agrees with it
  (humidity ±2%, temperature ±1 °C, within 5 minutes) it is accepted
  (`--- hum jump confirmed`), so real changes such as a storm pass after ~2 minutes while
  single noise packets are dropped. After more than 10 minutes (`STALE_TIMEOUT`) without a valid
  transmission any packet is accepted.

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
   are validated by channel (must match the configured channel), tail nibble `0xF`, `d[0]>>5 == d[4]&7`, the
   sensor ID filter (if set), temperature range (-30…60 °C) and humidity (≤ 100 %).

4. **Best-match** — among all candidates, the one with temperature closest
   to 29 °C is selected, with bonuses for matching last known sensor ID and
   bit offset and a penalty for humidity > 30 % (all of this biases against a
   neighbour sensor on the same channel). A candidate with the last known ID within
   ±5 °C of the last temperature is preferred. When a hard `sensorId` is configured,
   only matching candidates are considered.

5. **Smoothing** — humidity-jump confirmation (see *Scanner behaviour*), then the
   value is stored and published.

## Usage

### Flashing

1. Open `the-receiver.ino` in Arduino IDE
2. Select board: **NodeMCU 1.0 (ESP-12E Module)**
3. Install the **ESP8266** board package and the library **PubSubClient** by Nick O'Leary
4. Upload to the controller

Command line (`arduino-cli`):
```
arduino-cli config add board_manager.additional_urls https://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli core install esp8266:esp8266
arduino-cli lib install PubSubClient
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 .
arduino-cli upload  --fqbn esp8266:esp8266:nodemcuv2 -p COM3 .
```
Stop `capture_log.py` / the serial monitor before uploading, the port must be free.

### Serial port names and macOS commands

The examples use the Windows port `COM3`. On macOS (and Linux) use the device file instead:

| OS | Port | How to find it |
|----|------|----------------|
| Windows | `COM3` | Device Manager → Ports |
| macOS | `/dev/cu.usbserial-110` (or `/dev/cu.wchusbserial*`, `/dev/cu.usbmodem*`) | `ls /dev/cu.*` or `arduino-cli board list` |
| Linux | `/dev/ttyUSB0` | `ls /dev/ttyUSB*` (the user must be in the `dialout` group) |

On macOS use the `cu.` device, not `tty.`; the number in the name depends on the USB port and
adapter. The D1 Mini's CH340 usually needs no driver on current macOS.

Python tools (`capture_log.py`, `configure.py`): Homebrew's Python refuses a global
`pip install` ("externally-managed-environment"), so use a virtual environment once:

```
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial
```
(`.venv/` is a local folder, already listed in `.gitignore`. Run `source .venv/bin/activate` again in each new terminal.)

macOS variants of the commands:
```
PORT=/dev/cu.usbserial-110

# flash
arduino-cli upload --fqbn esp8266:esp8266:nodemcuv2 -p $PORT .

# configure over serial (interactive, or with options)
python configure.py $PORT
python configure.py $PORT --channel 2 --sensor-id 1D80 --yes

# capture the log to output.log
python capture_log.py $PORT

# just watch the log (exit: Ctrl+A then K in screen)
screen $PORT 115200
```
Only one program can hold the port: quit `screen` (or stop `capture_log.py`) before flashing
or running `configure.py`.

### First Boot — Configuration

On first boot (or after EEPROM reset) the ESP starts an access point:

1. Connect your phone/PC to WiFi SSID **the-receiver**
2. Open http://192.168.4.1
3. Enter your WiFi credentials and MQTT broker details
4. Optionally, set **Sensor ID** (hex) to lock the receiver to a specific
   sensor — use the `stable=0xXXXX` value from the serial log line `RF: CH2 id=0x... (stable=0x...)`.
   The active filter is printed at boot as `Sensor ID filter: 0x...`. Leave `0` for auto.
   Also pick the **Sensor channel** (CH1–CH3, default CH2); packets from other channels
   are ignored. It is printed at boot as `Sensor channel filter: CH2`.
5. Optionally, set the **NTP server** (empty = `pl.pool.ntp.org`) and the **GMT offset** in hours
   (e.g. `1`, `-5`, `5.5`; empty = Warsaw time, CET/CEST with automatic daylight saving).
   A fixed GMT offset has no daylight saving.
6. Click "Save & Reboot"

The device will then connect to your WiFi network, sync the clock over NTP and connect to the MQTT broker.

Startup order: WiFi → NTP sync (up to 15 s) → `Sensor ID filter` line → MQTT → RF decoding starts.
The clock is re-synced every 3 hours. Log timestamps `[hh:mm:ss.ms]` show local (Warsaw by default)
time once NTP is synced; until then (or if NTP fails) they show uptime.

### Configuration over serial (`configure.py`)

`configure.py` does the same as the AP page, without WiFi: it connects to the serial port,
shows the stored values, lets you edit them in a text dialog and, on SAVE, makes the firmware
write them to EEPROM and reboot. It works on a fresh board and on a configured one, at any time
(not only in AP mode).

Windows:
```
pip install pyserial windows-curses
python configure.py COM3
```

macOS (create the virtual environment once, see *Serial port names and macOS commands*):
```
python3 -m venv .venv && source .venv/bin/activate && pip install pyserial   # first time only
source .venv/bin/activate                                                    # every new terminal
python configure.py /dev/cu.usbserial-110
```

The dialog (needs a real terminal at least 60×19; `curses` is built into macOS/Linux Python,
Windows needs `windows-curses`):

```
 the-receiver configuration                         /dev/cu.usbserial-110
 Device: configured   (unsaved changes: 1)
 ---------------------------------------------------------------------
 >*WiFi SSID           [MyNet                                         ]
   WiFi password       [<set>                                         ]
   MQTT host           [mosquitto.local                               ]
   ...
   Sensor channel      [2                                             ]
   NTP server          [<empty>                                       ]
   GMT offset, hours   [<empty>                                       ]

   [ SAVE ]  [ EXIT ]
```

| Key | Action |
|-----|--------|
| Up / Down, Tab / Shift+Tab | Move between the fields and the SAVE / EXIT buttons |
| Enter | Edit the field (or press the selected button); on *Sensor channel* it switches 1→2→3 |
| Left / Right | In a field: move the cursor (channel: switch). On the buttons: SAVE ↔ EXIT |
| Enter / Esc while editing | Accept / cancel the edit (Ctrl+U clears the line, Home/End/Del work) |
| Del or Backspace | Clear the selected field (empty = default; for passwords: remove the stored one) |
| R | Revert the selected field to the stored value |
| F2 | Save (same as the SAVE button) |
| Esc, F10 | Exit (same as the EXIT button); asks first when there are unsaved changes |

- Changed fields are marked with `*`. Passwords are never sent back by the device: the field
  shows `<set>`, a new password is typed from scratch and shown as `***`; Enter on an empty
  password keeps the stored one.
- WiFi SSID and MQTT host are required. The device checks every value on SAVE (e.g. channel 5
  or a bad hex ID is rejected); the dialog then jumps to the offending field and shows the reason.
- Nothing is written before SAVE. After a successful SAVE the dialog closes, the device
  reboots and its boot log is shown for 15 s. EXIT leaves the device untouched.
- `--text` selects plain line prompts instead (Enter keeps the current value, `-` clears it).
  The script also falls back to them when `curses` is missing or the output is not a terminal.

Non-interactive mode changes only the options that are given (no dialog):

```
python configure.py COM3 --wifi-ssid MyNet --wifi-pass secret --mqtt-host 192.168.0.10 \
    --channel 2 --sensor-id 1D80 --yes
```

macOS:
```
python configure.py /dev/cu.usbserial-110 --wifi-ssid MyNet --wifi-pass secret \
    --mqtt-host 192.168.0.10 --channel 2 --sensor-id 1D80 --yes

# change a single field, keep everything else
python configure.py /dev/cu.usbserial-110 --channel 3 --yes

# clear the optional fields (NTP server and GMT offset back to defaults)
python configure.py /dev/cu.usbserial-110 --ntp "" --gmt "" --yes
```
In zsh/bash wrap a password containing `$ ! & ; "` or spaces in single quotes,
e.g. `--wifi-pass 'p@ss$word'`; a password with a single quote itself is easier to enter in the
dialog. `$PORT` works too (`PORT=/dev/cu.usbserial-110`).

| Option | Field |
|--------|-------|
| `--wifi-ssid`, `--wifi-pass` | WiFi credentials |
| `--mqtt-host`, `--mqtt-port`, `--mqtt-user`, `--mqtt-pass` | MQTT broker |
| `--sensor-id` | Sensor ID, hex, `0` = auto |
| `--channel` | Sensor channel 1–3 |
| `--ntp`, `--gmt` | NTP server, GMT offset in hours (`""` = Warsaw with DST) |
| `--yes` | Save without the final question |
| `--text` | Line prompts instead of the dialog |
| `--baud`, `--no-watch` | Baud rate (115200); skip the 15 s boot log shown after saving |

Notes:
- Opening the port normally resets the board. The firmware answers only from `loop()`, i.e.
  after WiFi and NTP at startup, so the script pings for up to 90 s.
- The port can be used by one program at a time: stop `capture_log.py` and the serial monitor first.
- Passwords given as options are visible in the process list and shell history; use the
  interactive mode on shared machines.
- Protocol: text lines starting with `CFG ` (`PING`, `GET`, `SET key=value`, `SAVE`, `CANCEL`);
  `SET` only stages a value, `SAVE` writes and reboots. Keys: `wifi_ssid wifi_pass mqtt_host
  mqtt_port mqtt_user mqtt_pass sensor_id channel ntp gmt`. Every reply carries the usual timestamp.

### Home Assistant

If MQTT Discovery is enabled in Home Assistant, two sensors
(temperature and humidity) appear automatically under device **the-receiver**.

**State topic:** `the-receiver/state` (JSON: `{"temperature":24.4,"humidity":57}`)  
**Availability topic:** `the-receiver/availability` (`online` / `offline`, retained)

The state is published every 2 minutes. After 10 minutes without a valid packet
the device is marked `offline`.

### Re-configuration

WiFi fail 10 times in a row (or WiFi unavailable at boot) → AP config mode automatically.
The AP is closed after 5 minutes without saving and the device reboots; RF decoding and MQTT
do not run while the AP is up.
To change the configuration without the AP, use `configure.py` (see above).
To reset it completely, clear EEPROM (e.g. by erasing flash via `esptool.py
erase_flash` or uploading an empty sketch).

### Serial Monitor

Output at 115200 baud:

```
=== the-receiver ===
[00:00:00.120] Config found in EEPROM
[00:00:00.125] Connecting WiFi to MyNetwork
[00:00:04.406] WiFi OK: 192.168.1.42
[00:00:04.410] NTP: syncing with pl.pool.ntp.org
[16:04:05.512] NTP: synced 2026-10-14 16:04:05 (UTC+2:00, Warsaw)
[16:04:05.514] Sensor ID filter: 0x1D80
[16:04:05.620] MQTT OK
```

Received RF packets are logged with sensor ID:
```
[14:23:45.123] RF: CH2 id=0x3D99 (stable=0x1D80)  24.5C  57%  raw:3D 99 0F 5F 39
```

The built-in blue LED blinks briefly on each valid RF packet (and every 10 s as a heartbeat).

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
- `*.log` files are listed in `.gitignore`.

## References

- [rtl_433](https://github.com/merbanan/rtl_433) — decoder for the 72-bit Geevon TX19-1 variant
  (`src/devices/geevon_tx19.c`, issue #3438 "Decoding Geevon TX19-1"); not the protocol of this device
