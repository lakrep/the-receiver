/*
  the-receiver — Geevon TX19 decoder for NodeMCU D1 Mini (ESP-12F)
  Protocol:
    40-bit packet: ID[16] | Temp_Cx10[8] | Flags[4] | Hum_H[4] | Hum_L[4] | Pad[4]
    Short space ~970us = 0, Long space ~1930us = 1, Mark ~550us

  Wiring: RX470C-V01 D0 → D2 (GPIO4) via 10k+22k divider, VCC→5V
*/

#define RF_PIN    D2
#define LED_PIN   LED_BUILTIN
#define MAX_PULSES 250

#define MARK_MIN    300
#define MARK_MAX    1000
#define SHORT_MIN   700
#define SHORT_MAX   1300
#define LONG_MIN    1500
#define LONG_MAX    2500
#define IDLE_TIMEOUT 10000
#define MAX_BITS    100

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

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println(F("\nthe-receiver — Geevon TX19"));
  Serial.println(F("ID  Temp  Hum\n"));
  pinMode(RF_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RF_PIN), handleInterrupt, CHANGE);
}

uint32_t lastHB = 0;

void loop() {
  if (millis() - lastHB > 10000) {
    lastHB = millis();
    digitalWrite(LED_PIN, LOW); delay(15); digitalWrite(LED_PIN, HIGH);
  }

  noInterrupts();
  uint32_t idle = micros() - lastTime;
  uint16_t count = pulseIndex;
  interrupts();

  if (count == 0 || idle < IDLE_TIMEOUT + 5000) return;

  uint16_t timings[MAX_PULSES];
  noInterrupts();
  memcpy(timings, (void*)pulses, count * sizeof(uint16_t));
  pulseIndex = 0;
  interrupts();

  if (count < 15) return;

  uint8_t bits[MAX_BITS];
  int bitCount = 0, valid = 0, invalid = 0;

  for (int i = 1; i < count - 1; i += 2) {
    uint16_t m = timings[i];
    uint16_t s = timings[i + 1];
    if (m < MARK_MIN || m > MARK_MAX || s < SHORT_MIN) { invalid++; continue; }
    if (s >= SHORT_MIN && s <= SHORT_MAX) {
      if (bitCount < MAX_BITS) bits[bitCount++] = 0;
    } else if (s >= LONG_MIN && s <= LONG_MAX) {
      if (bitCount < MAX_BITS) bits[bitCount++] = 1;
    } else { invalid++; continue; }
    valid++;
  }

  if (valid < 30 || invalid > valid) return;

  // Extract bytes from first 40 bits
  if (bitCount < 40) return;

  uint8_t d[5] = {0};
  for (int b = 0; b < 40; b++) {
    d[b / 8] = (d[b / 8] << 1) | (bits[b] & 1);
  }

  digitalWrite(LED_PIN, LOW);

  uint16_t id = ((uint16_t)d[0] << 8) | d[1];
  bool flagOk = ((d[3] >> 4) == 0x0F);
  bool notNoise = (d[2] != 0xFF && id != 0xFFFF);

  if (!flagOk || !notNoise) return;

  digitalWrite(LED_PIN, LOW);

  float tempC = d[2] * 0.1f;
  uint8_t hum = ((d[3] & 0x0F) << 4) | (d[4] >> 4);
  if (hum > 100) return;

  // Channel: d[1] upper nibble → 0x8=CH1, 0x9=CH2, 0xA=CH3
  uint8_t ch = (d[1] >> 4) - 7;
  if (ch < 1 || ch > 3) ch = 1;
  bool battLow = (d[1] & 0x08) != 0;

  unsigned long now = millis();
  char ts[14];
  snprintf(ts, sizeof(ts), "%02u:%02u:%02u.%03u",
    (unsigned int)((now / 3600000) % 24),
    (unsigned int)((now / 60000) % 60),
    (unsigned int)((now / 1000) % 60),
    (unsigned int)(now % 1000));
  Serial.print(F("["));
  Serial.print(ts);
  Serial.print(F("]  CH"));
  Serial.print(ch);
  Serial.print(battLow ? F(" !BAT") : F("    "));
  Serial.print(F("  "));
  if (tempC < 10) Serial.print(F(" "));
  Serial.print(tempC, 1);
  Serial.print(F("C  "));
  Serial.print(hum);
  Serial.print(F("%"));
  Serial.println();

  delay(50);
  digitalWrite(LED_PIN, HIGH);
}
