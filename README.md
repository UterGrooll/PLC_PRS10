# 📡 Arduino Modbus TCP Контроллер

**Универсальный контроллер с Modbus TCP, датчиком температуры DS18B20, двумя реле и тремя дискретными входами**

Данный проект — это готовый Modbus TCP-устройство на базе Arduino + W5500, позволяющее:

* Управлять двумя реле (одно с таймером авто-отключения)
* Считывать температуру с DS18B20
* Передавать состояние трёх дискретных входов (с антидребезгом)
* Работать по сети 24/7 без зависаний
* Быть полностью совместимым с любой SCADA (MasterSCADA / Modbus Poll / Codesys / etc.)

---

## 🛠 Возможности

### 🔌 **Modbus TCP сервер**

Устройство работает как сервер по стандартному порту **502**.
SCADA обращается к нему по IP и читает/пишет регистры.

### 🔥 **Реле**

* **Relay 1** — включается через Modbus, автоматически выключается через **60 секунд**.
* **Relay 2** — обычное управление (ON/OFF).

### 🌡 **Температура**

* Поддерживается любой датчик DS18B20
* Период обновления ~1 сек
* Значение передаётся в десятой доле градуса (например 253 → 25.3°C)

### ⚡ **Дискретные входы**

3 входа на пинах **D2, D3, D4**

* подключены как `INPUT_PULLUP`
* передают состояние:

  * 1 — контакты замкнуты (LOW)
  * 0 — разомкнуты (HIGH)

Входы оснащены **50 мс антидребезгом**.

### 💾 **Отказоустойчивость**

* Полностью без `delay()`
* Основные процессы работают через `millis()`
* Ethernet не блокируется
* Modbus обслуживается в режиме реального времени

---

# 🔧 Схема подключения

| Элемент    | Пин Arduino |
| ---------- | ----------- |
| DS18B20    | D9          |
| Relay 1    | D7          |
| Relay 2    | D6          |
| DI1        | D2          |
| DI2        | D3          |
| DI3        | D4          |
| W5500 (CS) | D10         |

Остальные пины SPI — стандартные.

---

# 📘 Modbus регистры

## **Holding Registers (Read/Write)**

| Адрес   | Назначение                | Формат |
| ------- | ------------------------- | ------ |
| **110** | Relay 1 (1 – ON, 0 – OFF) | R/W    |
| **111** | Relay 2 (1 – ON, 0 – OFF) | R/W    |
| **120** | Температура ×10           | R      |
| **130** | DI1                       | R      |
| **131** | DI2                       | R      |
| **132** | DI3                       | R      |

---

# 💻 Полный рабочий скетч

*(Уже проверен, стабильно работает в SCADA)*

```cpp
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

```

---

# ⚙ Настройки в SCADA / Modbus Poll

| Параметр            | Значение          |
| ------------------- | ----------------- |
| Mode                | Modbus TCP/IP     |
| Server IP           | **192.168.0.179** |
| Port                | **502**           |
| Response Timeout    | 1000 ms           |
| Delay Between Polls | 20–50 ms          |
| Connect Timeout     | 8000 ms           |

---

# 🚀 Рекомендации по эксплуатации

* Использовать отдельный MAC-адрес для каждого устройства
* Устанавливать статический IP
* Запитывать Arduino и W5500 от **стабильного 5V ≥ 1A**
* Не использовать длинные переменные delay() в любых частях программы

---

# 📝 Лицензия

Проект свободен к использованию, модификации и интеграции в любые SCADA-системы.

---
be UterGrooll 
2025
