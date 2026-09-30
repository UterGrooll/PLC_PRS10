#include <SPI.h>
#include <Ethernet.h>
#include "ModbusTCP_RU.h"
#include <GyverDS18.h>

/* ---------- Network & Modbus ---------- */
byte mac[] = {0x02, 0x47, 0xA1, 0x10, 0x00, 0x02};
IPAddress ip(192, 168, 1, 179);
IPAddress gateway(192, 168, 1, 1);
IPAddress dnsServer(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

ModbusTCP_RU mb;

/* ---------- Relays: Coils, FC01/FC05/FC15 ---------- */
const uint8_t RELAY1_PIN = 7;
const uint8_t RELAY2_PIN = 6;

const word COIL_RELAY1 = 0;
const word COIL_RELAY2 = 1;

const uint32_t RELAY1_TIMEOUT = 60000UL;  // 1 min
uint32_t relay1OnTime = 0;
bool relay1Active = false;

/* ---------- Temperature: Input Register, FC04 ---------- */
const uint8_t DS_PIN = 9;
GyverDS18Single ds(DS_PIN);

const word IREG_TEMP_X10 = 0;  // temperature * 10
const int16_t TEMP_ERROR_X10 = 32767;  // invalid/unavailable, not a temperature
const uint32_t TEMP_PERIOD_MS = 1000UL;  // >= 750 ms conversion at 12 bits
uint32_t temperatureRequestTime = 0;
bool temperatureConversionPending = false;

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

uint32_t linkCheckTimer = 0;
uint32_t linkUpTime = 0;
bool linkWasDown = false;
bool linkRecovering = false;

static_assert(MB_MAX_COILS > COIL_RELAY2, "Coil map is too small");
static_assert(MB_MAX_DISCRETE >= DISC_INPUT_BASE + INPUT_COUNT, "Discrete map is too small");
static_assert(MB_MAX_INPUT > IREG_TEMP_X10, "Input register map is too small");

void startEthernet()
{
  Ethernet.begin(mac, ip, dnsServer, gateway, subnet);
  mb.begin();
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
    linkWasDown = true;
    linkRecovering = false;
    return;
  }

  // W5100 reports Unknown; do not periodically reset an otherwise working chip.
  if (link == Unknown) {
    linkRecovering = false;
    return;
  }

  if (link == LinkON && linkWasDown) {
    if (!linkRecovering) {
      linkRecovering = true;
      linkUpTime = now;
    }
    if (now - linkUpTime >= LINK_RECOVER_DELAY_MS) {
      linkWasDown = false;
      linkRecovering = false;
      mb.restart();
    }
  }
}

void applyRelay(word address, bool value)
{
  switch (address) {
    case COIL_RELAY1:
      // Repeated SCADA writes must not postpone automatic shutdown.
      if (value && !relay1Active) {
        relay1OnTime = millis();
      } else if (!value) {
        relay1OnTime = 0;
      }
      relay1Active = value;
      digitalWrite(RELAY1_PIN, value ? HIGH : LOW);
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

    for (uint8_t i = 0; i < INPUT_COUNT; i++) {
      mb.Discrete(DISC_INPUT_BASE + i, (inputState >> i) & 1);
    }
  }
}

void updateRelayTimer(uint32_t now)
{
  if (relay1Active && (uint32_t)(now - relay1OnTime) >= RELAY1_TIMEOUT) {
    mb.setCoilLocal(COIL_RELAY1, false);
    applyRelay(COIL_RELAY1, false);
  }
}

void startTemperatureConversion()
{
  temperatureConversionPending = ds.requestTemp();
  temperatureRequestTime = millis();
  if (!temperatureConversionPending) {
    mb.Ireg(IREG_TEMP_X10, (word)TEMP_ERROR_X10);
  }
}

void updateTemperature()
{
  // Independent timer: a failed request must not prevent the next retry.
  if ((uint32_t)(millis() - temperatureRequestTime) < TEMP_PERIOD_MS) {
    return;
  }

  if (temperatureConversionPending && ds.readTemp()) {
    float temperature = ds.getTemp();
    int16_t tempX10 = (int16_t)(temperature * 10.0f);
    mb.Ireg(IREG_TEMP_X10, (word)tempX10);
  } else {
    mb.Ireg(IREG_TEMP_X10, (word)TEMP_ERROR_X10);
  }

  startTemperatureConversion();
}

void setup()
{
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

  mb.setCoilLocal(COIL_RELAY1, false);
  mb.setCoilLocal(COIL_RELAY2, false);
  mb.Ireg(IREG_TEMP_X10, (word)TEMP_ERROR_X10);

  for (uint8_t i = 0; i < INPUT_COUNT; i++) {
    pinMode(IN_PINS[i], INPUT_PULLUP);
    mb.Discrete(DISC_INPUT_BASE + i, false);
  }

  inputState = readInputsRaw();
  inputLast = inputState;

  for (uint8_t i = 0; i < INPUT_COUNT; i++) {
    mb.Discrete(DISC_INPUT_BASE + i, (inputState >> i) & 1);
  }

  mb.onCoilWrite(applyRelay);

  Ethernet.init(10);
  startEthernet();

  startTemperatureConversion();
}

void loop()
{
  mb.MbsRun();

  uint32_t now = millis();
  updateRelayTimer(now);
  updateEthernetWatchdog();
  updateTemperature();
  updateInputs(now);
}
