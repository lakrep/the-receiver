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
- OOK space-encoded: mark ~300-1000µs, space ~700-1100µs = 0, space ~1500-5000µs = 1
- Timing constants: `SPACE_MIN=700`, `SPACE_SHORT_MAX=1100`, `SPACE_LONG_MIN=1500`, `SPACE_MAX=5000`

### Packet layout
```
Byte 0    Byte 1    Byte 2    Byte 3    Byte 4
IIIIIIII  IICC?HHH  TTTTTTTT  TTTThhhh  hhhhhhhh
```
- **Byte 0:** ID high byte
- **Byte 1:** Bits 7-4 = channel-7 (8=CH1, 9=CH2, A=CH3), Bits 3-2 = battery(bit3) + flags
- **Byte 2-3:** Temperature 12-bit — `((d[2]<<4 | d[3]>>4)) * 0.1°C` (NO -500 offset)
- **Byte 4:** Humidity %

### Decoding
- `channel = (d[1] >> 4) - 7;`
- `tempC = (((int)d[2] << 4) | (d[3] >> 4)) * 0.1f;`
- `hum = d[4];`
- No checksum verified yet.
- Scanner iterates offsets & inversions, filters CH2, tracks ID + offset for lock-on.
- **Hard sensor ID filter:** if `cfg.sensorId != 0`, only packets with `(id & ID_MASK) == cfg.sensorId` are accepted — all others are silently discarded.
  `ID_MASK = 0x1FC0`: top 3 bits of d[0] drift between packets (same sensor shows 1D99/3D99/…/FD99), d[1] low bits are battery/flags.
  `sensorId` is masked on save and on load, so a legacy value like `9D99` behaves as `1D80`.
- **Tail check:** `(d[3] & 0x0F) == 0x0F` required (held for 1778/1778 good packets; corrupted tails often break it).
- **Humidity smoothing:** a jump greater than ±5% (`HUM_JUMP_MAX`) holds the packet back (`--- hum jump pending`, nothing
  published, lock-on state untouched). The next packet confirms it if humidity differs ≤2% and temperature ≤1 °C
  from the held one within `PENDING_TIMEOUT` (5 min) → accepted (`--- hum jump confirmed`). Single noise packets
  never get through; real changes (a 2026-10-14 storm: 71→80% humidity, 13.7→11 °C in 10 min, rejected for 8 min
  by the old reject-only filter) pass after 2 packets. After 10 minutes (`STALE_TIMEOUT`) without a valid packet any packet is accepted.
- **Station clips humidity <20% to 20% and >95% to 95%.**
  Per the station spec: *"The indoor and outdoor humidity range is 20%-95%"*.
  TX19 sensor sends true 15-21% but the base station's display doesn't go below
  20%. Our decoder reports the real d[4] value.
- **Neighbor CH2 rejection:** Another sensor on CH2 (23°C, 51% hum) is in range.
  Scanner uses `if (hum > 30) delta += 20` to bias against high-humidity candidates.
  Temperature consistency (±5°C fresh, ±10°C stale) uses `lastTempC != 0` instead
  of `hasData`, persisting across stale timeouts to prevent wrong-sensor capture.

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
- Single `.ino` file, EEPROM stores WiFi+MQTT+config in `Storage` struct (magic 0xDEAD)
  - Fields: `magic`, `wifiSSID`, `wifiPass`, `mqttHost`, `mqttPort`, `mqttUser`, `mqttPass`, `sensorId`
  - `sensorId` is `0` (auto) by default; when non-zero only packets matching that ID are accepted
- First boot (or cleared EEPROM): AP mode "the-receiver" → http://192.168.4.1
- Config portal fields: WiFi SSID/Pass, MQTT Host/Port/User/Pass, Sensor ID (hex, 0 = auto), NTP server, GMT offset (hours)
  - `Storage` also has `ntpServer[40]` (empty = `pl.pool.ntp.org`), `tzCustom`, `gmtOffsetMin`; unwritten/invalid values fall back to defaults on load
  - Empty/unparsable GMT offset → Warsaw time computed in code (`localOffsetSec()`: CEST UTC+2 between last Sunday of March and October 01:00 UTC, else CET UTC+1); a number → fixed offset, no DST. libc `TZ`/`localtime` is NOT used (DST was not applied on the target core). System clock stays UTC (`configTime(0, 0, server)`)
- Startup order in `setup()`: WiFi → `syncTime()` (waits ≤15 s) → print `Sensor ID filter` → MQTT → `attachInterrupt` (RF polling)
- NTP re-sync every 3 h via `sntp_update_delay_MS_rfc_not_less_than_15000()` override (`NTP_RESYNC_MS`); (weak lwIP hook)
- Serial log shows `RF: CH2 id=0xXXXX (stable=0xYYYY)` — copy the **stable** value into the web form to lock the receiver to one sensor
- Boot log prints `Sensor ID filter: 0x...` (or `off (auto)`) — check it to confirm the saved value
- WiFi reconnect: 10s interval, 10 failures → fallback to AP
- Publish interval: 120s, stale timeout: 600s
- RF decode runs in interrupt, non-blocking LED timer replaces `delay()`
- Every output to serial port requires timing in format [hh:mm:ss.ms] — local (Warsaw) wall clock once NTP is valid (`time() > MIN_VALID_EPOCH`), uptime before that
