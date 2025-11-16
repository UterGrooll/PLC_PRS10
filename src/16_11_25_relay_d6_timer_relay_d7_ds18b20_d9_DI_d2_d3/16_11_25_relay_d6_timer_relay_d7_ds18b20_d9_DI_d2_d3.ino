#include <SPI.h>
#include <Ethernet.h>
#include <ModbusEthernet.h>
#include <GyverDS18.h>

/* ---------- Network & Modbus ---------- */
byte mac[] = { 0xDE, 0xDD, 0xBE, 0xEF, 0xFE, 0x01 };
IPAddress ip(192, 168, 0, 179);
ModbusEthernet mb;

/* ---------- Relays ---------- */
const uint8_t RELAY1_PIN = 7;
const uint8_t RELAY2_PIN = 6;
const uint16_t REG_RELAY1 = 110;
const uint16_t REG_RELAY2 = 111;
const uint32_t RELAY1_TIMEOUT = 60000UL;  // 1 min
uint32_t relay1OnTime = 0;

/* ---------- Temperature ---------- */
const uint8_t DS_PIN = 9;
GyverDS18Single ds(DS_PIN);
const uint16_t REG_TEMP = 120;

/* ---------- Inputs ---------- */
const uint8_t IN_PINS[] = { 2, 3, 4 };
const uint16_t REG_IN_BASE = 130;
const uint32_t DEBOUNCE_MS = 50;
uint8_t inputState = 0xFF;  // 1=closed (LOW), 0=open (HIGH)
uint8_t inputLast  = 0xFF;
uint32_t debounceTimer = 0;

/* ---------- Serial debug ---------- */
uint32_t serialTimer = 0;
const uint32_t SERIAL_PERIOD = 500;

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(115200);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

  for (uint8_t i = 0; i < 3; ++i) {
    pinMode(IN_PINS[i], INPUT_PULLUP);
    mb.addReg(HREG(REG_IN_BASE + i));
    mb.Hreg(REG_IN_BASE + i, 0);
  }

  Ethernet.init(10);
  Ethernet.begin(mac, ip);
  mb.server();

  mb.addReg(HREG(REG_RELAY1));
  mb.Hreg(REG_RELAY1, 0);
  mb.addReg(HREG(REG_RELAY2));
  mb.Hreg(REG_RELAY2, 0);
  mb.addReg(HREG(REG_TEMP));
  mb.Hreg(REG_TEMP, 0);

  ds.requestTemp();

  Serial.println(F("System Ready!"));
}

/* ---------- Main loop ---------- */
void loop() {
  mb.task();  // handle Modbus requests
  uint32_t now = millis();

  /* ===== Relay 1 with timer ===== */
  if (mb.Hreg(REG_RELAY1)) {
    digitalWrite(RELAY1_PIN, HIGH);
    if (!relay1OnTime) relay1OnTime = now;
    if ((now - relay1OnTime) >= RELAY1_TIMEOUT) {
      mb.Hreg(REG_RELAY1, 0);
      relay1OnTime = 0;
      digitalWrite(RELAY1_PIN, LOW);
    }
  } else {
    relay1OnTime = 0;
    digitalWrite(RELAY1_PIN, LOW);
  }

  /* ===== Relay 2 ===== */
  digitalWrite(RELAY2_PIN, mb.Hreg(REG_RELAY2) ? HIGH : LOW);

  /* ===== Temperature ===== */
  if (ds.ready()) {
    if (ds.readTemp()) {
      float t = ds.getTemp();
      mb.Hreg(REG_TEMP, (int)(t * 10));  // 1/10 °C precision
    }
    ds.requestTemp();  // schedule next reading
  }

  /* ===== Dry contacts with debounce ===== */
  uint8_t nowInputs = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    if (digitalRead(IN_PINS[i]) == LOW) nowInputs |= (1 << i);
  }

  if (nowInputs != inputLast) {
    debounceTimer = now;
    inputLast = nowInputs;
  } else if ((now - debounceTimer) >= DEBOUNCE_MS && nowInputs != inputState) {
    inputState = nowInputs;
    for (uint8_t i = 0; i < 3; ++i) {
      mb.Hreg(REG_IN_BASE + i, (inputState >> i) & 1);
    }
  }

  /* ===== Serial debug (non‑blocking) ===== */
  if ((now - serialTimer) >= SERIAL_PERIOD) {
    serialTimer = now;
    Serial.print(F("Temp="));
    Serial.print(mb.Hreg(REG_TEMP) / 10.0);
    Serial.print(F("C R1="));
    Serial.print(mb.Hreg(REG_RELAY1));
    Serial.print(F(" R2="));
    Serial.print(mb.Hreg(REG_RELAY2));
    Serial.print(F(" IN="));
    Serial.println(inputState, BIN);
  }
}
