#include <SPI.h>
#include <Ethernet.h>
#include "ModbusTCP_RU.h"
#include <GyverDS18.h>

extern EthernetServer MbServer;

/* ---------- Network & Modbus ---------- */
byte mac[] = { 0xDE, 0xDD, 0xBE, 0xEF, 0xFE, 0x01 };
IPAddress ip(192, 168, 1, 179);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

ModbusTCP_RU mb;

/* ---------- Relays: Coils, FC01/FC05/FC15 ---------- */
const uint8_t RELAY1_PIN = 7;
const uint8_t RELAY2_PIN = 6;

const word COIL_RELAY1 = 0;
const word COIL_RELAY2 = 1;

const uint32_t RELAY1_TIMEOUT = 60000UL;
uint32_t relay1OnTime = 0;

/* ---------- Temperature: Input Register, FC04 ---------- */
const uint8_t DS_PIN = 9;
GyverDS18Single ds(DS_PIN);

const word IREG_TEMP_X10 = 0;

/* ---------- Inputs: Discrete Inputs, FC02 ---------- */
const uint8_t IN_PINS[] = { 2, 3, 4, 5 };
const uint8_t INPUT_COUNT = sizeof(IN_PINS) / sizeof(IN_PINS[0]);

const word DISC_INPUT_BASE = 0;

const uint32_t DEBOUNCE_MS = 50;
uint8_t inputState = 0;
uint8_t inputLast = 0;
uint32_t debounceTimer = 0;

/* ---------- Ethernet watchdog ---------- */
const uint32_t LINK_CHECK_PERIOD_MS = 1000UL;
const uint32_t LINK_RECOVER_DELAY_MS = 1500UL;
const uint32_t FORCE_REINIT_PERIOD_MS = 30000UL;

uint32_t linkCheckTimer = 0;
uint32_t linkDownTime = 0;
uint32_t lastEthernetRestart = 0;
bool linkWasDown = false;

void restartEthernet()
{
  Ethernet.begin(mac, ip, gateway, subnet);
  delay(500);
  MbServer.begin();
  lastEthernetRestart = millis();
}

void updateEthernetWatchdog()
{
  uint32_t now = millis();

  if (now - linkCheckTimer < LINK_CHECK_PERIOD_MS) {
    return;
  }
  linkCheckTimer = now;

  EthernetLinkStatus link = Ethernet.linkStatus();

  if (link == LinkOFF) {
    if (!linkWasDown) {
      linkWasDown = true;
      linkDownTime = now;
    }
    return;
  }

  if (link == LinkON && linkWasDown) {
    if (now - linkDownTime >= LINK_RECOVER_DELAY_MS) {
      linkWasDown = false;
      restartEthernet();
    }
    return;
  }

  if (link == Unknown && (now - lastEthernetRestart >= FORCE_REINIT_PERIOD_MS)) {
    restartEthernet();
  }
}

void applyRelay(word address, bool value)
{
  switch (address) {
    case COIL_RELAY1:
      digitalWrite(RELAY1_PIN, value ? HIGH : LOW);

      if (value) {
        relay1OnTime = millis();
      } else {
        relay1OnTime = 0;
      }
      break;

    case COIL_RELAY2:
      digitalWrite(RELAY2_PIN, value ? HIGH : LOW);
      break;
  }
}

uint8_t readInputsRaw()
{
  uint8_t value = 0;

  for (uint8_t i = 0; i < INPUT_COUNT; i++) {
    if (digitalRead(IN_PINS[i]) == LOW) {
      value |= (1 << i);
    }
  }

  return value;
}

void publishInputs()
{
  for (uint8_t i = 0; i < INPUT_COUNT; i++) {
    mb.Discrete(DISC_INPUT_BASE + i, (inputState >> i) & 1);
  }
}

void updateInputs(uint32_t now)
{
  uint8_t nowInputs = readInputsRaw();

  if (nowInputs != inputLast) {
    inputLast = nowInputs;
    debounceTimer = now;
    return;
  }

  if ((now - debounceTimer) >= DEBOUNCE_MS && nowInputs != inputState) {
    inputState = nowInputs;
    publishInputs();
  }
}

void updateRelayTimer(uint32_t now)
{
  if (!mb.Coil(COIL_RELAY1)) {
    relay1OnTime = 0;
    return;
  }

  if (relay1OnTime == 0) {
    relay1OnTime = now;
  }

  if ((now - relay1OnTime) >= RELAY1_TIMEOUT) {
    mb.Coil(COIL_RELAY1, false);
  }
}

void updateTemperature()
{
  if (!ds.ready()) {
    return;
  }

  if (ds.readTemp()) {
    int16_t tempX10 = (int16_t)(ds.getTemp() * 10.0f);
    mb.Ireg(IREG_TEMP_X10, (word)tempX10);
  }

  ds.requestTemp();
}

void setup()
{
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

  mb.Coil(COIL_RELAY1, false);
  mb.Coil(COIL_RELAY2, false);
  mb.Ireg(IREG_TEMP_X10, 0);

  for (uint8_t i = 0; i < INPUT_COUNT; i++) {
    pinMode(IN_PINS[i], INPUT_PULLUP);
    mb.Discrete(DISC_INPUT_BASE + i, false);
  }

  inputState = readInputsRaw();
  inputLast = inputState;
  publishInputs();

  mb.onCoilWrite(applyRelay);

  Ethernet.init(10);
  restartEthernet();

  ds.requestTemp();
}

void loop()
{
  mb.MbsRun();

  uint32_t now = millis();

  updateEthernetWatchdog();
  updateRelayTimer(now);
  updateTemperature();
  updateInputs(now);
}
