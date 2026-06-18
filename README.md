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
| Short space  | 700–1300 µs   | Bit 0   |
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
| ID          | d[0]–d[1]   | 16   | Sensor ID (0x7D99, 0xDD99, etc.) |
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

**Temperature formula confirmed:**
- Raw 12-bit = `(d[2] << 4) | (d[3] >> 4)` (e.g. `0x118` = 280 → 28.0 °C)
- NO -500 offset (TX19-1 uses offset, this device does not)

**Humidity confirmed:** `d[4]` only (not split across d[3]/d[4]).

## Algorithm

1. **GPIO interrupt** captures every pin state change (CHANGE mode) and records
   the microsecond timestamp. A gap >5 ms (`IDLE_TIMEOUT`) resets the pulse
   buffer, treating the next pulses as a new packet.

2. **Bit extraction** — marks (carrier) and spaces are paired:
   - `mark` 300–1000 µs + `space` 700–1300 µs → **bit 0**
   - `mark` 300–1000 µs + `space` 1500–5000 µs → **bit 1**
   - Anything else → invalid

3. **40-bit scan** — every possible offset and inversion is tested. Candidates
   are validated by channel nibble (must be 8/9/A) and temperature range.

4. **Best-match** — among all CH2 candidates, the one with temperature closest
   to 29 °C is selected, with bonuses for matching last known sensor ID and
   bit offset (prevents drift to neighbour sensors on the same channel).

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
4. Click "Save & Reboot"

The device will then connect to your WiFi network and the MQTT broker.

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
Config found in EEPROM
Connecting WiFi to MyNetwork
WiFi OK: 192.168.1.42
MQTT OK
```

The built-in blue LED blinks briefly on each valid RF packet.

## References

- [rtl_433](https://github.com/merbanan/rtl_433) — Geevon TX19 decoder (issue #3438)
