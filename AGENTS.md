# the-receiver — AGENTS.md

## Board & Environment
- **Board:** NodeMCU D1 Mini (ESP-12F), Arduino IDE
- **Libraries:** PubSubClient (Nick O'Leary), ESP8266WiFi, ESP8266WebServer
- **Flash:** select "NodeMCU 1.0 (ESP-12E Module)" in Arduino IDE

## Hardware
- RX470C-V01 on D2 (GPIO4) via **10kΩ+22kΩ voltage divider** (5V→3.3V)
- VCC→5V (not 3.3V), antenna 17.3cm quarter-wave
- LED_BUILTIN (GPIO2), active LOW

## RF Protocol (Geevon TX19)
- 40-bit OOK PWM, mark ~550µs
- Short space ~970µs = **0**, long ~1930µs = **1**
- Timing constants: `SPACE_MIN`, `SPACE_SHORT_MAX`, `SPACE_LONG_MIN`, `SPACE_MAX` — do NOT use `LONG_MIN`/`LONG_MAX` (conflict with `<limits.h>`)

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
