/*
  the-receiver — Geevon TX19 433MHz decoder with WiFi + MQTT + Home Assistant
  Board: NodeMCU D1 Mini (ESP-12F)

  // Обновлено: 40-битный (5 байт). Температура: 12-bit (d[2]<<4 | d[3]>>4) * 0.1. Влажность: d[4] или комбинированная.
  // Канал: (d[1]>>4) - 7 (8=CH1, 9=CH2, A=CH3). Батарея: d[1] bit 3.

  First boot: AP mode "the-receiver" → http://192.168.4.1 → configure WiFi + MQTT
  Subsequent boots: read from EEPROM, connect WiFi → MQTT → HA Discovery
*/

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>

// ============================================================
// Pin definitions
// ============================================================
#define RF_PIN      D2
#define LED_PIN     LED_BUILTIN

// ============================================================
// RF decoder timing
// ============================================================
#define MAX_PULSES    250
#define MARK_MIN      300
#define MARK_MAX      1000
#define SPACE_MIN     700
#define SPACE_SHORT_MAX 1300
#define SPACE_LONG_MIN 1500
#define SPACE_MAX     5000
#define IDLE_TIMEOUT  5000
#define MAX_BITS      100
#define VALID_MIN     30
#define PACKET_BITS   40
#define PACKET_BYTES  5

// ============================================================
// EEPROM configuration
// ============================================================
#define EEPROM_SIZE   512
#define CFG_MAGIC     0xDEAD

struct Storage {
  uint16_t magic;
  char wifiSSID[32];
  char wifiPass[64];
  char mqttHost[64];
  uint16_t mqttPort;
  char mqttUser[32];
  char mqttPass[32];
};

// ============================================================
// Timing constants
// ============================================================
#define PUBLISH_INTERVAL  120000   // 2 min between MQTT publishes
#define STALE_TIMEOUT     600000   // 10 min → unavailable
#define RECONNECT_DELAY   10000    // 10 s between WiFi/MQTT retries
#define CONFIG_TIMEOUT    300000   // 5 min in AP mode before reboot

#define SERIAL_PRINT(...)  do { if (!silent) { Serial.__VA_ARGS__; } } while(0)

// ============================================================
// Global state
// ============================================================
Storage cfg;
bool configured = false;
bool webConfigMode = false;
bool silent = true;
unsigned long lastConnectTry = 0;
unsigned long lastPublish = 0;
unsigned long lastPacket = 0;
unsigned long configStart = 0;

float lastTempC = 0;
uint8_t lastHum = 0;
bool hasData = false;
unsigned long ledOffTime = 0;

ESP8266WebServer http(80);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ============================================================
// RF decoder globals
// ============================================================
volatile uint16_t pulses[MAX_PULSES];
volatile uint16_t pulseIndex = 0;
volatile uint32_t lastTime = 0;

void IRAM_ATTR handleInterrupt() {
  uint32_t now = micros();
  uint32_t dur = now - lastTime;
  lastTime = now;
  if (dur > IDLE_TIMEOUT) pulseIndex = 0;
  if (pulseIndex < MAX_PULSES)
    pulses[pulseIndex++] = (uint16_t)(dur > 65535 ? 65535 : dur);
}

// ============================================================
// EEPROM helpers
// ============================================================
void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  uint8_t *p = (uint8_t *)&cfg;
  for (unsigned int i = 0; i < sizeof(cfg); i++)
    p[i] = EEPROM.read(i);
  EEPROM.end();
  configured = (cfg.magic == CFG_MAGIC);
}

void saveConfig() {
  cfg.magic = CFG_MAGIC;
  EEPROM.begin(EEPROM_SIZE);
  uint8_t *p = (uint8_t *)&cfg;
  for (unsigned int i = 0; i < sizeof(cfg); i++)
    EEPROM.write(i, p[i]);
  EEPROM.commit();
  EEPROM.end();
  configured = true;
}

// ============================================================
// WiFi connection
// ============================================================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  printTimestamp();
  SERIAL_PRINT(print(F("Connecting WiFi to ")));
  SERIAL_PRINT(println(cfg.wifiSSID));
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.wifiSSID, cfg.wifiPass);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    printTimestamp();
    SERIAL_PRINT(print(F("WiFi OK: ")));
    SERIAL_PRINT(println(WiFi.localIP()));
  } else {
    printTimestamp();
    SERIAL_PRINT(println(F("WiFi FAILED")));
  }
}

// ============================================================
// MQTT
// ============================================================
void mqttCallback(char *topic, byte *payload, unsigned int len) {
  // no commands expected, but keep the callback
}

bool connectMQTT() {
  if (mqtt.connected()) return true;
  mqtt.setBufferSize(512);
  mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(60);

  char clientId[32];
  snprintf(clientId, sizeof(clientId), "the-receiver-%06X", ESP.getChipId());

  bool ok;
  if (cfg.mqttUser[0])
    ok = mqtt.connect(clientId, cfg.mqttUser, cfg.mqttPass, "the-receiver/availability", 1, true, "offline");
  else
    ok = mqtt.connect(clientId, NULL, NULL, "the-receiver/availability", 1, true, "offline");

  if (ok) {
    mqtt.publish("the-receiver/availability", "online", true);
    publishDiscovery();
    printTimestamp();
    SERIAL_PRINT(println(F("MQTT OK")));
  } else {
    printTimestamp();
    SERIAL_PRINT(print(F("MQTT FAIL: ")));
    SERIAL_PRINT(println(mqtt.state()));
  }
  return ok;
}

// ============================================================
// Home Assistant MQTT Discovery
// ============================================================
void publishDiscovery() {
  char buf[512];
  char topic[80];
  char chipId[16];
  snprintf(chipId, sizeof(chipId), "%06X", ESP.getChipId());

  snprintf(topic, sizeof(topic), "homeassistant/sensor/receiver-temp-%s/config", chipId);
  snprintf(buf, sizeof(buf),
    "{\"name\":\"Temperature\",\"stat_t\":\"the-receiver/state\","
    "\"unit_of_meas\":\"\\u00b0C\",\"dev_cla\":\"temperature\","
    "\"uniq_id\":\"receiver-temp-%s\","
    "\"avty_t\":\"the-receiver/availability\","
    "\"val_tpl\":\"{{ value_json.temperature }}\","
    "\"device\":{\"identifiers\":[\"the-receiver-%s\"],"
    "\"name\":\"the-receiver\",\"model\":\"Geevon TX19 Decoder\","
    "\"manufacturer\":\"DIY\"}}",
    chipId, chipId);
  mqtt.publish(topic, buf, true);

  snprintf(topic, sizeof(topic), "homeassistant/sensor/receiver-hum-%s/config", chipId);
  snprintf(buf, sizeof(buf),
    "{\"name\":\"Humidity\",\"stat_t\":\"the-receiver/state\","
    "\"unit_of_meas\":\"%%\",\"dev_cla\":\"humidity\","
    "\"uniq_id\":\"receiver-hum-%s\","
    "\"avty_t\":\"the-receiver/availability\","
    "\"val_tpl\":\"{{ value_json.humidity }}\","
    "\"device\":{\"identifiers\":[\"the-receiver-%s\"],"
    "\"name\":\"the-receiver\"}}",
    chipId, chipId);
  mqtt.publish(topic, buf, true);
}

void publishState() {
  if (!mqtt.connected()) return;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"temperature\":%.1f,\"humidity\":%u}", lastTempC, lastHum);
  mqtt.publish("the-receiver/state", buf, false);
  printTimestamp();
  SERIAL_PRINT(print(F("MQTT publish: ")));
  SERIAL_PRINT(println(buf));
}

// ============================================================
// Web config portal
// ============================================================
void handleRoot() {
  String page = F(
    "<!DOCTYPE html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>the-receiver Setup</title><style>"
    "body{font-family:sans-serif;max-width:500px;margin:auto;padding:20px}"
    "input{width:100%;padding:8px;margin:6px 0;box-sizing:border-box}"
    "label{font-weight:bold}.h{font-size:24px;margin-bottom:20px}"
    "</style></head><body>"
    "<div class=h>the-receiver</div>"
    "<form action=/save method=POST>"
    "<label>WiFi SSID</label><input name=s required>"
    "<label>WiFi Password</label><input name=p type=password required>"
    "<label>MQTT Host</label><input name=mh required>"
    "<label>MQTT Port</label><input name=mp value=1883>"
    "<label>MQTT Username</label><input name=mu>"
    "<label>MQTT Password</label><input name=mpw type=password>"
    "<br><br><input type=submit value='Save & Reboot' style='background:#4CAF50;color:#fff;border:0;padding:12px;font-size:16px'>"
    "</form></body></html>");
  http.send(200, "text/html", page);
}

void handleSave() {
  strlcpy(cfg.wifiSSID, http.arg("s").c_str(), sizeof(cfg.wifiSSID));
  strlcpy(cfg.wifiPass, http.arg("p").c_str(), sizeof(cfg.wifiPass));
  strlcpy(cfg.mqttHost, http.arg("mh").c_str(), sizeof(cfg.mqttHost));
  cfg.mqttPort = (uint16_t)http.arg("mp").toInt();
  if (cfg.mqttPort == 0) cfg.mqttPort = 1883;
  strlcpy(cfg.mqttUser, http.arg("mu").c_str(), sizeof(cfg.mqttUser));
  strlcpy(cfg.mqttPass, http.arg("mpw").c_str(), sizeof(cfg.mqttPass));
  saveConfig();
  http.send(200, "text/html", F("<html><body><h2>Saved! Rebooting...</h2></body></html>"));
  delay(500);
  ESP.restart();
}

void startConfigPortal() {
  webConfigMode = true;
  configStart = millis();
  WiFi.mode(WIFI_AP);
  WiFi.softAP("the-receiver", NULL, 1, false, 4);
  http.on("/", handleRoot);
  http.on("/save", HTTP_POST, handleSave);
  http.begin();
  SERIAL_PRINT(println(F("\nAP mode: the-receiver")));
  SERIAL_PRINT(println(WiFi.softAPIP()));
}

void handleConfigClient() {
  if (!webConfigMode) return;
  http.handleClient();

  // LED pattern: 3 short (100ms on, 200ms off), then 1000ms pause
  static int step = 0;
  static unsigned long lastStep = 0;
  unsigned long now = millis();
  int durations[] = {100, 200, 100, 200, 100, 1000};
  if (now - lastStep >= (unsigned long)durations[step]) {
    lastStep = now;
    step = (step + 1) % 6;
    digitalWrite(LED_PIN, (step % 2 == 0) ? LOW : HIGH);
  }

  if (millis() - configStart > CONFIG_TIMEOUT) {
    SERIAL_PRINT(println(F("Config timeout, rebooting...")));
    delay(100);
    ESP.restart();
  }
}

// ============================================================
// RF decoder
// ============================================================
void decodeRF() {
  uint16_t timings[MAX_PULSES];
  noInterrupts();
  uint32_t idle = micros() - lastTime;
  uint16_t count = pulseIndex;
  interrupts();

  if (count == 0 || idle < IDLE_TIMEOUT + 5000) return;

  noInterrupts();
  memcpy(timings, (void*)pulses, count * sizeof(uint16_t));
  pulseIndex = 0;
  interrupts();

  if (count < 10) return;

  uint8_t bits[MAX_BITS];
  int bitCount = 0, valid = 0, invalid = 0;
  for (int i = 1; i < count - 1; i += 2) {
    uint16_t m = timings[i];
    uint16_t s = timings[i + 1];
    if (m < MARK_MIN || m > MARK_MAX || s < SPACE_MIN) { invalid++; continue; }
    if (s <= SPACE_SHORT_MAX) {
      if (bitCount < MAX_BITS) bits[bitCount++] = 0;
    } else if (s >= SPACE_LONG_MIN && s <= SPACE_MAX) {
      if (bitCount < MAX_BITS) bits[bitCount++] = 1;
    } else { invalid++; continue; }
    valid++;
  }

  if (bitCount < PACKET_BITS) return;

  // --- 40-bit scan — pick best CH2 packet ---
  int bestOff = -1;
  uint8_t bestD[PACKET_BYTES];
  float bestTemp = 0;
  uint8_t bestHum = 0;
  int bestDelta = 9999;

  for (int off = 0; off + PACKET_BITS <= bitCount; off++) {
    for (int inv = 0; inv < 2; inv++) {
      uint8_t d[PACKET_BYTES] = {0};
      for (int b = 0; b < PACKET_BITS; b++)
        d[b / 8] = (d[b / 8] << 1) | ((bits[off + b] & 1) ^ inv);
      if (d[0] == 0xFF && d[1] == 0xFF) continue;
      uint8_t channel = (d[1] >> 4) - 7;
      if (channel != 2) continue;

      // try all temp/hum formula combos
      float temps[] = {
        d[2] + (d[3] >> 4),                    // t1
        ((int)d[2] << 4 | (d[3] >> 4)) * 0.1f // t2
      };
      uint8_t hums[] = {
        (uint8_t)(((d[3] & 0x0F) << 4) | (d[4] >> 4)), // h1
        d[4],                                           // h2 (byte 4)
      };

      for (int ti = 0; ti < 2; ti++) {
        for (int hi = 0; hi < 2; hi++) {
          if (temps[ti] < -30 || temps[ti] > 60) continue;
          if (hums[hi] > 100) continue;
          int delta = abs((int)(temps[ti] * 10) - 290);  // prefer 29C
          if (delta < bestDelta) {
            bestDelta = delta; bestOff = off;
            memcpy(bestD, d, PACKET_BYTES);
            bestTemp = temps[ti]; bestHum = hums[hi];
          }
        }
      }
    }
  }
  if (bestOff < 0) return;

  printTimestamp();
  SERIAL_PRINT(print(F("RF: CH2  ")));
  SERIAL_PRINT(print(bestTemp, 1));
  SERIAL_PRINT(print(F("C  ")));
  SERIAL_PRINT(print(bestHum));
  SERIAL_PRINT(print(F("%  raw:")));
  for (int i = 0; i < PACKET_BYTES; i++) { SERIAL_PRINT(print(bestD[i], HEX)); SERIAL_PRINT(print(" ")); }
  SERIAL_PRINT(println());

  lastTempC = bestTemp;
  lastHum = bestHum;
  lastPacket = millis();
  hasData = true;

  // Blink LED
  digitalWrite(LED_PIN, LOW);
  ledOffTime = millis() + 50;
}

void printTimestamp() {
  if (silent) return;
  unsigned long t = millis();
  unsigned long h = t / 3600000;
  unsigned long m = (t % 3600000) / 60000;
  unsigned long s = (t % 60000) / 1000;
  unsigned long ms = t % 1000;
  Serial.print(F("["));
  if (h < 10) Serial.print('0');
  Serial.print(h);
  Serial.print(':');
  if (m < 10) Serial.print('0');
  Serial.print(m);
  Serial.print(':');
  if (s < 10) Serial.print('0');
  Serial.print(s);
  Serial.print('.');
  if (ms < 100) Serial.print('0');
  if (ms < 10) Serial.print('0');
  Serial.print(ms);
  Serial.print(F("] "));
}

// ============================================================
// Setup / Loop
// ============================================================

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);
  unsigned long serialTimeout = millis() + 2000;
  while (!Serial && millis() < serialTimeout) {}
  silent = (millis() >= serialTimeout);

  if (!silent) Serial.println(F("\n=== the-receiver ==="));

  loadConfig();

  if (configured) {
    SERIAL_PRINT(println(F("Config found in EEPROM")));
    connectWiFi();
    if (WiFi.status() == WL_CONNECTED) {
      connectMQTT();
    } else {
      SERIAL_PRINT(println(F("WiFi failed — starting AP for reconfiguration")));
      startConfigPortal();
    }
  } else {
    SERIAL_PRINT(println(F("No config — starting AP")));
    startConfigPortal();
  }

  pinMode(RF_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RF_PIN), handleInterrupt, CHANGE);
}

void loop() {
  yield();
  // LED off timer (non-blocking replacement for delay)
  if (ledOffTime && millis() >= ledOffTime) {
    digitalWrite(LED_PIN, HIGH);
    ledOffTime = 0;
  }

  // Web config portal
  if (webConfigMode) {
    handleConfigClient();
    return; // skip everything else in config mode
  }

  // MQTT loop
  if (mqtt.connected()) mqtt.loop();

  // Reconnection attempt
  static uint8_t failCount = 0;
  if (millis() - lastConnectTry > RECONNECT_DELAY) {
    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
      if (WiFi.status() != WL_CONNECTED) {
        if (++failCount > 10) { // 10×10s = 100s of failures
          SERIAL_PRINT(println(F("Persistent WiFi failure — starting AP")));
          startConfigPortal();
        }
      } else {
        failCount = 0;
      }
    } else if (!mqtt.connected()) {
      connectMQTT();
    } else {
      failCount = 0;
    }
    lastConnectTry = millis();
  }

  // Heartbeat blink every 10 s
  static unsigned long lastHB = 0;
  if (millis() - lastHB > 10000) {
    lastHB = millis();
    digitalWrite(LED_PIN, LOW);
    ledOffTime = millis() + 15;
  }

  // RF decoder
  decodeRF();

  // MQTT publish every 2 min
  if (hasData && (lastPublish == 0 || millis() - lastPublish > PUBLISH_INTERVAL)) {
    lastPublish = millis();
    publishState();
    if (mqtt.connected()) {
      mqtt.publish("the-receiver/availability", "online", true);
    }
  }

  // Mark unavailable if stale
  if (hasData && millis() - lastPacket > STALE_TIMEOUT) {
    hasData = false;
    if (mqtt.connected())
      mqtt.publish("the-receiver/availability", "offline", true);
  }
}
