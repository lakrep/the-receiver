/*
  the-receiver — Geevon TX19 433MHz decoder with WiFi + MQTT + Home Assistant
  Board: NodeMCU D1 Mini (ESP-12F)

  40-bit (5 byte), temperature 12-bit (d[2]<<4 | d[3]>>4) * 0.1, humidity d[4],
  channel ((d[1]>>4) & 3) + 1 (bits 5-4 of d[1], as in rtl_433 TX19-1), battery unconfirmed.

  First boot: AP mode "the-receiver" → http://192.168.4.1 → configure WiFi + MQTT + NTP
  Subsequent boots: read from EEPROM, connect WiFi → NTP sync → print sensor ID →
  MQTT → HA Discovery → start RF decoding. NTP is re-synced every 3 hours.
  Local time is computed here (EU DST rules for Warsaw, or a fixed offset), not by libc TZ.
*/

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <time.h>
#include <sys/time.h>

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
#define SPACE_SHORT_MAX 1100
#define SPACE_LONG_MIN 1500
#define SPACE_MAX     5000
#define IDLE_TIMEOUT  5000
#define MAX_BITS      100
#define VALID_MIN     30
#define PACKET_BITS   40
#define PACKET_BYTES  5

// Sensor ID = d[0]<<8 | d[1]. Only bits 12..6 (d[0] low 5 bits + d[1] bits 7-6) are stable:
// the top 3 bits of d[0] drift between packets (seen 1D/3D/5D/7D/9D/BD/DD/FD for one sensor),
// d[1] low bits carry flags. Config and comparisons always use id & ID_MASK.
#define ID_MASK       0x1FC0

// ============================================================
// NTP / time zone
// ============================================================
#define NTP_DEFAULT_SERVER  "pl.pool.ntp.org"
#define NTP_SYNC_TIMEOUT    15000                         // ms to wait for the first sync
#define NTP_RESYNC_MS       (3UL * 60 * 60 * 1000)        // periodic re-sync
#define MIN_VALID_EPOCH     1700000000                    // anything below = clock not set

// ============================================================
// EEPROM configuration
// ============================================================
#define EEPROM_SIZE   512
#define CFG_MAGIC     0xDEAD
#define DEFAULT_CHANNEL 2

struct Storage {
  uint16_t magic;
  char wifiSSID[32];
  char wifiPass[64];
  char mqttHost[64];
  uint16_t mqttPort;
  char mqttUser[32];
  char mqttPass[32];
  uint16_t sensorId;
  char ntpServer[40];     // empty = NTP_DEFAULT_SERVER
  uint8_t tzCustom;       // 1 = use gmtOffsetMin, otherwise Warsaw (CET/CEST, EU DST rules)
  int16_t gmtOffsetMin;   // minutes east of UTC, valid only when tzCustom == 1
  uint8_t channel;        // sensor channel 1..3 to accept (anything else = DEFAULT_CHANNEL)
};

// ============================================================
// Timing constants
// ============================================================
#define PUBLISH_INTERVAL  120000   // 2 min between MQTT publishes
#define STALE_TIMEOUT     600000   // 10 min → unavailable
#define RECONNECT_DELAY   10000    // 10 s between WiFi/MQTT retries
#define CONFIG_TIMEOUT    300000   // 5 min in AP mode before reboot

// Humidity jump handling: a jump > HUM_JUMP_MAX is held back until the next packet confirms it
#define HUM_JUMP_MAX      5        // % between consecutive accepted packets
#define HUM_CONFIRM_TOL   2        // % allowed between the two confirming packets
#define TEMP_CONFIRM_TOL  1.0f     // °C allowed between the two confirming packets
#define PENDING_TIMEOUT   300000   // ms a held-back packet waits for confirmation

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
// keep only hostname characters; an invalid or unwritten value becomes "" (= default server)
void normalizeNtpServer() {
  cfg.ntpServer[sizeof(cfg.ntpServer) - 1] = 0;
  for (const char *c = cfg.ntpServer; *c; c++) {
    if (!isalnum((uint8_t)*c) && *c != '.' && *c != '-') { cfg.ntpServer[0] = 0; break; }
  }
}

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  uint8_t *p = (uint8_t *)&cfg;
  for (unsigned int i = 0; i < sizeof(cfg); i++)
    p[i] = EEPROM.read(i);
  EEPROM.end();
  configured = (cfg.magic == CFG_MAGIC);

  // fields added later read as 0xFF.. on EEPROM saved by older firmware
  normalizeNtpServer();
  if (cfg.tzCustom != 1 || cfg.gmtOffsetMin < -720 || cfg.gmtOffsetMin > 840) {
    cfg.tzCustom = 0;
    cfg.gmtOffsetMin = 0;
  }
  if (cfg.channel < 1 || cfg.channel > 3) cfg.channel = DEFAULT_CHANNEL;
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
// NTP time sync
// ============================================================
bool timeIsValid() {
  return time(nullptr) > MIN_VALID_EPOCH;
}

// lwIP SNTP calls this to schedule its periodic re-sync (default is 1 hour)
extern "C" uint32_t sntp_update_delay_MS_rfc_not_less_than_15000() {
  return NTP_RESYNC_MS;
}

// days since 1970-01-01 for a civil date (proleptic Gregorian)
long daysFromCivil(int y, int m, int d) {
  y -= m <= 2;
  long era = (y >= 0 ? y : y - 399) / 400;
  long yoe = y - era * 400;
  long doy = (153L * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

// 01:00 UTC on the last Sunday of the given month (March or October, both have 31 days)
time_t lastSundayUtc(int year, int month) {
  long days = daysFromCivil(year, month, 31);
  int dow = (int)((days + 4) % 7);  // 1970-01-01 was Thursday; 0 = Sunday
  return (time_t)(days - dow) * 86400 + 3600;
}

// seconds to add to UTC to get local time
int localOffsetSec(time_t utc) {
  if (cfg.tzCustom) return cfg.gmtOffsetMin * 60;
  struct tm t;
  gmtime_r(&utc, &t);
  int year = t.tm_year + 1900;
  bool dst = utc >= lastSundayUtc(year, 3) && utc < lastSundayUtc(year, 10);
  return dst ? 7200 : 3600;  // Warsaw: CEST = UTC+2, CET = UTC+1
}

void toLocal(time_t utc, struct tm *out) {
  time_t local = utc + localOffsetSec(utc);
  gmtime_r(&local, out);
}

void syncTime() {
  const char *server = cfg.ntpServer[0] ? cfg.ntpServer : NTP_DEFAULT_SERVER;

  printTimestamp();
  SERIAL_PRINT(print(F("NTP: syncing with ")));
  SERIAL_PRINT(println(server));

  configTime(0L, 0, server);  // system clock stays UTC, local time is derived in toLocal()
  unsigned long start = millis();
  while (!timeIsValid() && millis() - start < NTP_SYNC_TIMEOUT) {
    delay(100);
  }

  printTimestamp();
  if (timeIsValid()) {
    time_t now = time(nullptr);
    struct tm t;
    toLocal(now, &t);
    int off = localOffsetSec(now) / 60;
    char buf[40];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d (UTC%+d:%02d%s)",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec,
             off / 60, abs(off) % 60, cfg.tzCustom ? "" : ", Warsaw");
    SERIAL_PRINT(print(F("NTP: synced ")));
    SERIAL_PRINT(println(buf));
  } else {
    SERIAL_PRINT(println(F("NTP: sync failed, retrying in background (log shows uptime until then)")));
  }
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
  char sid[8];
  snprintf(sid, sizeof(sid), "%X", cfg.sensorId);
  String gmt;
  if (cfg.tzCustom) {
    gmt = (cfg.gmtOffsetMin % 60 == 0) ? String(cfg.gmtOffsetMin / 60) : String(cfg.gmtOffsetMin / 60.0f, 2);
  }
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
    "<label>Sensor ID (hex, 0 = auto)</label><input name=sid value=");
  page += sid;
  page += F(" placeholder='e.g. 1D80'>"
    "<small>Use the stable= value from the serial log. Only bits 0x1FC0 are compared.</small>"
    "<label>Sensor channel</label><select name=ch style='width:100%;padding:8px;margin:6px 0'>");
  for (int c = 1; c <= 3; c++) {
    page += F("<option value=");
    page += c;
    if (c == cfg.channel) page += F(" selected");
    page += F(">CH");
    page += c;
    page += F("</option>");
  }
  page += F("</select>"
    "<label>NTP server (empty = " NTP_DEFAULT_SERVER ")</label><input name=ntp value='");
  page += cfg.ntpServer;
  page += F("' placeholder='" NTP_DEFAULT_SERVER "'>"
    "<label>GMT offset, hours (empty = Warsaw, CET/CEST)</label><input name=gmt value='");
  page += gmt;
  page += F("' placeholder='e.g. 1, -5, 5.5'>"
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
  cfg.sensorId = (uint16_t)strtol(http.arg("sid").c_str(), NULL, 16) & ID_MASK;
  cfg.channel = (uint8_t)http.arg("ch").toInt();
  if (cfg.channel < 1 || cfg.channel > 3) cfg.channel = DEFAULT_CHANNEL;

  String ntp = http.arg("ntp");
  ntp.trim();
  strlcpy(cfg.ntpServer, ntp.c_str(), sizeof(cfg.ntpServer));
  normalizeNtpServer();

  // empty or unparsable offset = default Warsaw time with DST
  String gmt = http.arg("gmt");
  gmt.trim();
  gmt.replace(',', '.');
  cfg.tzCustom = 0;
  cfg.gmtOffsetMin = 0;
  if (gmt.length()) {
    char *end;
    double h = strtod(gmt.c_str(), &end);
    if (end != gmt.c_str() && *end == 0 && h >= -12 && h <= 14) {
      cfg.tzCustom = 1;
      cfg.gmtOffsetMin = (int16_t)(h * 60 + (h < 0 ? -0.5 : 0.5));
    }
  }
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
  int bitCount = 0;
  for (int i = 1; i < count - 1; i += 2) {
    uint16_t m = timings[i];
    uint16_t s = timings[i + 1];
    if (m < MARK_MIN || m > MARK_MAX || s < SPACE_MIN) continue;
    if (s <= SPACE_SHORT_MAX) {
      if (bitCount < MAX_BITS) bits[bitCount++] = 0;
    } else if (s >= SPACE_LONG_MIN && s <= SPACE_MAX) {
      if (bitCount < MAX_BITS) bits[bitCount++] = 1;
    }
  }

  if (bitCount < PACKET_BITS) return;

  // --- 40-bit scan — pick best packet on the configured channel matching our sensor ID ---
  int bestOff = -1;
  uint8_t bestD[PACKET_BYTES];
  float bestTemp = 0;
  uint8_t bestHum = 0;
  int bestDelta = 9999;
  int bestIdOff = -1;
  uint8_t bestIdD[PACKET_BYTES];
  float bestIdTemp = 0;
  uint8_t bestIdHum = 0;
  int bestIdDelta = 9999;
  static uint16_t lastGoodId = 0;
  static int lastGoodOff = -1;

  for (int off = 0; off + PACKET_BITS <= bitCount; off++) {
    for (int inv = 0; inv < 2; inv++) {
      uint8_t d[PACKET_BYTES] = {0};
      for (int b = 0; b < PACKET_BITS; b++)
        d[b / 8] = (d[b / 8] << 1) | ((bits[off + b] & 1) ^ inv);
      if (d[0] == 0xFF && d[1] == 0xFF) continue;
      uint8_t channel = ((d[1] >> 4) & 3) + 1;
      if (channel != cfg.channel) continue;
      uint16_t id = ((uint16_t)d[0] << 8) | d[1];

      // d[3] low nibble was 0xF in all 1778 good packets of a 41 h capture, never in corrupted tails
      if ((d[3] & 0x0F) != 0x0F) continue;

      // d[0] top 3 bits copy the humidity low 3 bits (97% of 1872 packets); a mismatch means corruption
      if ((d[0] >> 5) != (d[4] & 7)) continue;

      // hard sensor ID filter (ID_MASK drops the drifting top bits and channel/battery flags)
      if (cfg.sensorId != 0 && (id & ID_MASK) != cfg.sensorId) continue;

      // temp from 12-bit formula only
      float tempC = (((int)d[2] << 4) | (d[3] >> 4)) * 0.1f;
      if (tempC < -30 || tempC > 60) continue;

      // humidity: byte 4 only
      uint8_t hum = d[4];
      if (hum > 100) continue;

      int delta = abs((int)(tempC * 10) - 290);
      if (lastGoodId && (id & ID_MASK) == (lastGoodId & ID_MASK)) delta -= 30;
      if (lastGoodOff >= 0 && off == lastGoodOff) delta -= 20;
      if (hum > 30) delta += 20;

      // track best overall
      if (delta < bestDelta) {
        bestDelta = delta; bestOff = off;
        memcpy(bestD, d, PACKET_BYTES);
        bestTemp = tempC; bestHum = hum;
      }
      // track best matching known ID (fallback)
      if (lastGoodId && (id & ID_MASK) == (lastGoodId & ID_MASK) && tempC < lastTempC + 5.0f && tempC > lastTempC - 5.0f) {
        if (delta < bestIdDelta) {
          bestIdDelta = delta; bestIdOff = off;
          memcpy(bestIdD, d, PACKET_BYTES);
          bestIdTemp = tempC; bestIdHum = hum;
        }
      }
    }
  }

  // prefer ID-matched packet within ±5C; fall back to best overall
  if (bestIdOff >= 0) {
    bestOff = bestIdOff; memcpy(bestD, bestIdD, PACKET_BYTES);
    bestTemp = bestIdTemp; bestHum = bestIdHum;
  } else if (lastTempC != 0.0f && bestOff >= 0) {
    float maxDiff = hasData ? 5.0f : 10.0f;
    if (bestTemp > lastTempC + maxDiff || bestTemp < lastTempC - maxDiff)
      return; // skip — no consistent packet found
  }
  if (bestOff < 0) return;
  uint16_t bestId = ((uint16_t)bestD[0] << 8) | bestD[1];

  printTimestamp();
  SERIAL_PRINT(print(F("RF: CH")));
  SERIAL_PRINT(print(cfg.channel));
  SERIAL_PRINT(print(F(" id=0x")));
  SERIAL_PRINT(print(bestId, HEX));
  SERIAL_PRINT(print(F(" (stable=0x")));
  SERIAL_PRINT(print(bestId & ID_MASK, HEX));
  SERIAL_PRINT(print(F(")  ")));
  SERIAL_PRINT(print(bestTemp, 1));
  SERIAL_PRINT(print(F("C  ")));
  SERIAL_PRINT(print(bestHum));
  SERIAL_PRINT(print(F("%  raw:")));
  for (int i = 0; i < PACKET_BYTES; i++) { if (bestD[i] < 0x10) SERIAL_PRINT(print("0")); SERIAL_PRINT(print(bestD[i], HEX)); SERIAL_PRINT(print(" ")); }
  SERIAL_PRINT(println());

  // humidity smoothing: a jump > HUM_JUMP_MAX is not accepted at once (noise, wrong sensor),
  // but a second packet that agrees with it (real weather change, e.g. a storm) is.
  // Stale data (> STALE_TIMEOUT) is accepted without any check.
  static bool hasPending = false;
  static float pendTemp = 0;
  static uint8_t pendHum = 0;
  static unsigned long pendTime = 0;
  if (hasData && millis() - lastPacket < STALE_TIMEOUT) {
    int diff = (int)bestHum - (int)lastHum;
    if (diff > HUM_JUMP_MAX || diff < -HUM_JUMP_MAX) {
      bool confirmed = hasPending && millis() - pendTime < PENDING_TIMEOUT &&
                       abs((int)bestHum - (int)pendHum) <= HUM_CONFIRM_TOL &&
                       fabsf(bestTemp - pendTemp) <= TEMP_CONFIRM_TOL;
      printTimestamp();
      SERIAL_PRINT(print(confirmed ? F("--- hum jump confirmed: ") : F("--- hum jump pending: ")));
      SERIAL_PRINT(print(bestTemp, 1));
      SERIAL_PRINT(print(F("C ")));
      SERIAL_PRINT(print(bestHum));
      SERIAL_PRINT(println(F("%")));
      if (!confirmed) {
        hasPending = true;
        pendTemp = bestTemp;
        pendHum = bestHum;
        pendTime = millis();
        return;  // hold back, do not touch lock-on state or published values
      }
    }
  }
  hasPending = false;

  // accepted: only now remember ID/offset, so rejected packets cannot steer the lock-on
  lastGoodOff = bestOff;
  lastGoodId = bestId;
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
  unsigned long h, m, s, ms;
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec > MIN_VALID_EPOCH) {
    // NTP-synced local (Warsaw by default) wall clock
    struct tm t;
    toLocal(tv.tv_sec, &t);
    h = t.tm_hour; m = t.tm_min; s = t.tm_sec;
    ms = tv.tv_usec / 1000;
  } else {
    // not synced yet: uptime
    unsigned long up = millis();
    h = up / 3600000;
    m = (up % 3600000) / 60000;
    s = (up % 60000) / 1000;
    ms = up % 1000;
  }
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

  // fix uninitialized sensorId (unwritten EEPROM reads 0xFFFF)
  if (cfg.sensorId == 0xFFFF) cfg.sensorId = 0;
  cfg.sensorId &= ID_MASK;  // values saved before masking (e.g. 9D99) become 1D80

  // order: WiFi -> NTP sync -> sensor ID -> MQTT -> RF decoding
  if (configured) {
    printTimestamp();
    SERIAL_PRINT(println(F("Config found in EEPROM")));
    connectWiFi();
    if (WiFi.status() == WL_CONNECTED) syncTime();
  }

  printTimestamp();
  SERIAL_PRINT(print(F("Sensor channel filter: CH")));
  SERIAL_PRINT(println(cfg.channel));

  printTimestamp();
  if (cfg.sensorId) {
    SERIAL_PRINT(print(F("Sensor ID filter: 0x")));
    SERIAL_PRINT(println(cfg.sensorId, HEX));
  } else {
    SERIAL_PRINT(println(F("Sensor ID filter: off (auto)")));
  }

  if (configured && WiFi.status() == WL_CONNECTED) {
    connectMQTT();
  } else if (configured) {
    printTimestamp();
    SERIAL_PRINT(println(F("WiFi failed — starting AP for reconfiguration")));
    startConfigPortal();
  } else {
    printTimestamp();
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
