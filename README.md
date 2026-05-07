# PLC_PRS10

`PLC_PRS10` — Arduino Modbus TCP Slave контроллер для SCADA.

Проект построен на Arduino + Ethernet W5100/W5500 и библиотеке [`ModbusTCP_RU`](https://github.com/UterGrooll/ModbusTCP_RU). Устройство предоставляет SCADA две релейные команды, температуру DS18B20 и четыре дискретных входа.

![Фото устройства](https://github.com/UterGrooll/PLC_PRS10/blob/main/screenshot/20251112_151544.jpg)

## Назначение

Контроллер предназначен для небольших задач автоматизации:

- управление двумя реле из SCADA;
- автоматическое отключение первого реле через 60 секунд;
- чтение температуры с датчика DS18B20;
- чтение четырёх дискретных входов;
- работа по Modbus TCP через порт `502`;
- восстановление Ethernet-сервера после выдёргивания и возврата патч-корда.

## Аппаратная часть

| Узел | Пин Arduino |
| --- | --- |
| Relay 1 | D7 |
| Relay 2 | D6 |
| DI1 | D2 |
| DI2 | D3 |
| DI3 | D4 |
| DI4 | D5 |
| DS18B20 | D9 |
| W5100/W5500 CS | D10 |

Остальные пины SPI используются стандартно для Ethernet-модуля.

Схема:

![Схема](https://github.com/UterGrooll/PLC_PRS10/blob/main/screenshot/shematic.png)

## Сеть

По умолчанию в скетче задано:

```cpp
byte mac[] = { 0xDE, 0xDD, 0xBE, 0xEF, 0xFE, 0x01 };
IPAddress ip(192, 168, 1, 179);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
```

SCADA подключается к:

| Параметр | Значение |
| --- | --- |
| Mode | Modbus TCP/IP |
| Server IP | `192.168.1.179` |
| Port | `502` |
| Slave ID | `1` |
| Timeout | `1000 ms` |
| Scan Rate | `1000 ms` |

## Карта Modbus

В проекте используется раздельная Modbus-модель из `ModbusTCP_RU`.

### Coils

Чтение: `FC01`
Запись: `FC05` или `FC15`

| Address | Назначение | Формат |
| --- | --- | --- |
| `0` | Relay 1 | `0 = OFF`, `1 = ON` |
| `1` | Relay 2 | `0 = OFF`, `1 = ON` |

Relay 1 автоматически отключается через `60` секунд после включения.

### Discrete Inputs

Чтение: `FC02`

| Address | Назначение | Формат |
| --- | --- | --- |
| `0` | DI1, пин D2 | `1 = замкнут`, `0 = разомкнут` |
| `1` | DI2, пин D3 | `1 = замкнут`, `0 = разомкнут` |
| `2` | DI3, пин D4 | `1 = замкнут`, `0 = разомкнут` |
| `3` | DI4, пин D5 | `1 = замкнут`, `0 = разомкнут` |

Входы работают через `INPUT_PULLUP`, поэтому активное состояние — `LOW`.

### Input Registers

Чтение: `FC04`

| Address | Назначение | Формат |
| --- | --- | --- |
| `0` | Температура DS18B20 | `°C * 10` |

Пример: значение `253` означает `25.3 °C`.

## Настройка Modbus Poll

Для проверки реле:

| Параметр | Значение |
| --- | --- |
| Function | `01 Read Coils` |
| Address | `0` |
| Quantity | `2` |

Для записи реле:

| Параметр | Relay 1 | Relay 2 |
| --- | --- | --- |
| Function | `05 Write Single Coil` | `05 Write Single Coil` |
| Address | `0` | `1` |
| Value | `ON/OFF` | `ON/OFF` |

Для входов:

| Параметр | Значение |
| --- | --- |
| Function | `02 Read Discrete Inputs` |
| Address | `0` |
| Quantity | `4` |

Для температуры:

| Параметр | Значение |
| --- | --- |
| Function | `04 Read Input Registers` |
| Address | `0` |
| Quantity | `1` |

![Modbus Poll](https://github.com/UterGrooll/PLC_PRS10/blob/main/screenshot/Modbus%20Poll.png)

![Rapid SCADA](https://github.com/UterGrooll/PLC_PRS10/blob/main/screenshot/Rapid%20Scada.png)

## Ethernet Watchdog

В скетч добавлен watchdog Ethernet-соединения.

Если патч-корд вытащили и затем подключили обратно, W5100/W5500 иногда отвечает на ping, но TCP-сервер Modbus не принимает соединение до перезагрузки Arduino. Watchdog отслеживает возврат `LinkON` и выполняет:

```cpp
Ethernet.begin(mac, ip, gateway, subnet);
MbServer.begin();
```

Это восстанавливает Modbus TCP Server без ручной перезагрузки платы.

## Основной скетч

Актуальный скетч находится здесь:

[`src/17_11_25_relay_d6_relay_d7_ds18b20_d9_DI_d2_d3_d4_d5/17_11_25_relay_d6_relay_d7_ds18b20_d9_DI_d2_d3_d4_d5.ino`](src/17_11_25_relay_d6_relay_d7_ds18b20_d9_DI_d2_d3_d4_d5/17_11_25_relay_d6_relay_d7_ds18b20_d9_DI_d2_d3_d4_d5.ino)

## Зависимости

- `SPI`
- `Ethernet`
- [`ModbusTCP_RU`](https://github.com/UterGrooll/ModbusTCP_RU)
- `GyverDS18`

## Лицензия

Проект свободен для использования, модификации и интеграции в SCADA-системы.

---

by UterGrooll
2026
