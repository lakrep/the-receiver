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

Reverse-engineered from live captures. The sensor uses **OOK PWM**
(On-Off Keying, Pulse Width Modulation) with the following measured timings:

| Symbol       | Duration     | Meaning        |
|--------------|-------------|----------------|
| Mark (carrier) | ~550 µs    | Constant pulse width |
| Short space  | ~970 µs     | Bit 0         |
| Long space   | ~1930 µs    | Bit 1         |

### Packet format (40 bits)

```
Byte 0      Byte 1      Byte 2      Byte 3      Byte 4
[ID high]   [ID low ]   [Temp   ]   [Flags ]   [Humidity]
                    +chan       x0.1°C    +hum H     +trailer
```

| Field       | Position   | Size | Description                  |
|-------------|------------|------|------------------------------|
| ID          | bits 0–15  | 16   | Sensor ID (0x7580, 0x6C90, etc.) |
| Temperature | bits 16–23 | 8    | °C × 10 (0–25.5 °C)         |
| Flags       | bits 24–27 | 4    | Always 0xF                   |
| Humidity H  | bits 28–31 | 4    | High nibble of humidity      |
| Humidity L  | bits 32–35 | 4    | Low nibble of humidity       |
| Trailer     | bits 36–39 | 4    | CRC / channel indicator      |

**Channel** is encoded in the upper nibble of byte 1:
- `0x80` → CH1
- `0x90` → CH2
- `0xA0` → CH3

Therefore `channel = (d[1] >> 4) - 7`.

**Battery** is bit 3 of byte 1 (`d[1] & 0x08`).

## Algorithm

1. **GPIO interrupt** captures every pin state change (CHANGE mode) and records
   the microsecond timestamp. A gap >10 ms resets the pulse buffer, treating
   the next pulses as a new packet.

2. **Pulse pairing** — the interrupt buffer contains alternating HIGH/LOW
   durations. Pairs are formed as `(mark, space)` starting from the first pulse
   after the idle gap. The first pulse (idle time) is discarded.

3. **Symbol decoding** — each pair is classified:
   - `mark` 300–1000 µs + `space` 700–1300 µs → **bit 0**
   - `mark` 300–1000 µs + `space` 1500–2500 µs → **bit 1**
   - Anything else → invalid (noise)

4. **Validation** — if the number of valid pairs ≥ 30 and exceeds invalid
   pairs, the first 40 bits are packed into 5 bytes and validated:
   - Nibble 3 upper must be `0xF` (protocol sync marker)
   - Not all-1s noise (`d[2] != 0xFF`, `id != 0xFFFF`)

5. **Output** — temperature = `d[2] × 0.1`, humidity = `(d[3] & 0x0F) << 4) | (d[4] >> 4)`,
   channel = `(d[1] >> 4) - 7`.

## Usage

1. Open `the-receiver.ino` in Arduino IDE
2. Select board: **NodeMCU 1.0 (ESP-12E Module)**
3. Upload to the controller
4. Open Serial Monitor at **115200 baud**
5. Output appears as:

```
[12:34:56.789]  CH1       24.4C  57%
[12:35:46.805]  CH2       22.1C  63%
```

The blue LED on the NodeMCU blinks briefly on each valid packet.

## References

- [rtl_433](https://github.com/merbanan/rtl_433) — Geevon TX19 decoder (issue #3438)
- [rc-switch](https://github.com/sui77/rc-switch) — generic 433 MHz library (not used here; protocol is custom)
