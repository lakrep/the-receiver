# the-receiver — AGENTS.md

## Board & Environment
- **Board:** NodeMCU D1 Mini (ESP-12F), Arduino IDE
- **Libraries:** PubSubClient (Nick O'Leary), ESP8266WiFi, ESP8266WebServer
- **Flash:** select "NodeMCU 1.0 (ESP-12E Module)" in Arduino IDE

## Hardware
- RX470C-V01 on D2 (GPIO4) via **10kΩ+22kΩ voltage divider** (5V→3.3V)
- VCC→5V (not 3.3V), antenna 17.3cm quarter-wave
- LED_BUILTIN (GPIO2), active LOW

## RF Protocol (Geevon TX19 no LCD — this device)
- **40-bit (5 byte)** packet (NOT 72-bit TX19-1). Bits NOT inverted (inv=1 for CH2, inv=0 for CH1).
- OOK space-encoded: mark ~300-1000µs, space ~700-1300µs = 0, space ~1500-5000µs = 1
- Timing constants: `SPACE_MIN=700`, `SPACE_SHORT_MAX=1300`, `SPACE_LONG_MIN=1500`, `SPACE_MAX=5000`

### Packet layout
```
Byte 0    Byte 1    Byte 2    Byte 3    Byte 4
IIIIIIII  IICC?HHH  TTTTTTTT  TTTThhhh  hhhhhhhh
```
- **Byte 0:** ID high byte
- **Byte 1:** Bits 7-4 = channel-7 (8=CH1, 9=CH2, A=CH3), Bits 3-2 = battery(bit3) + flags
- **Byte 2-3:** Temperature 12-bit — `((d[2]<<4 | d[3]>>4)) * 0.1°C` (NO -500 offset)
- **Byte 3-4:** Humidity — `((d[3]&0x0F)<<4) | (d[4]>>4)` OR just `d[4]` — scanner tries both

### Decoding
- `channel = (d[1] >> 4) - 7;`
- `tempC = (((int)d[2] << 4) | (d[3] >> 4)) * 0.1f;`
- `hum = ((d[3] & 0x0F) << 4) | (d[4] >> 4);` — fallback: `hum = d[4];`
- No checksum verified yet.
- Scanner tries both inversions (inv=0, inv=1) and picks CH2 with temp closest to 29°C.

## RF Protocol (Geevon TX19-1 with LCD — reference only, NOT used)
- 72-bit (9 byte), bits inverted, LFSR-8 checksum. See `rtl_433/src/devices/geevon_tx19.c`.

## MQTT
- State topic: `the-receiver/state` → `{"temperature":24.4,"humidity":57}` (no retain)
- Availability: `the-receiver/availability` → `online`/`offline` (retained)
- **Must call `mqtt.setBufferSize(512)`** in `connectMQTT()` — discovery payloads are ~370B, PubSubClient default is 256B
- **Must call `mqtt.setKeepAlive(60)`** — 15s default causes timeout on ESP8266

## Home Assistant Discovery
- Topic: `homeassistant/sensor/receiver-{temp,hum}-{chipId}/config`
- Payload includes `dev_cla`, `unit_of_meas`, `val_tpl`, device grouping
- Published with `retain=true` on MQTT connect

## ESP8266 Quirks
- Call `yield()` at top of `loop()` for WiFi stack
- Use `SERIAL_PRINT(...)` macro (guarded by `silent` flag) instead of raw `Serial.print*`
- Silent mode: 2s `while(!Serial)` timeout in `setup()`
- LED blink pattern in AP mode: 3×100ms + 1000ms pause state machine

## Key Architecture
- Single `.ino` file, EEPROM stores WiFi+MQTT config in `Storage` struct (magic 0xDEAD)
- First boot: AP mode "the-receiver" → http://192.168.4.1
- WiFi reconnect: 10s interval, 10 failures → fallback to AP
- Publish interval: 120s, stale timeout: 600s
- RF decode runs in interrupt, non-blocking LED timer replaces `delay()`
- Every output to serial port requires timing in format [hh:mm:ss.ms]
