# SHT40_DS28E18 Library

A PlatformIO/Arduino library for the **SHT4x** temperature and humidity sensor connected via the **DS28E18** 1-Wire to I2C bridge.

This library allows you to use SHT4x sensors (like SHT40) over long distances using the 1-Wire protocol, mediated by the DS28E18 bridge and a DS2482 I2C-to-1-Wire master.

## Features

- Full support for SHT4x measurement commands via DS28E18 sequencer.
- CRC validation for temperature and humidity data.
- Support for multiple sensors on the same 1-Wire bus.
- Compatible with Raspberry Pi Pico, ESP32, and other Arduino-supported platforms.

## Dependencies

This library depends on:
- [DS28E18_DS2482_Library](https://github.com/DanielSart/DS28E18_DS2482_Library) - DS28E18 Bridge & Sequencer logic.
- [Adafruit DS248x](https://github.com/adafruit/Adafruit_DS248x) - DS2482 I2C to 1-Wire Master.

## Hardware Setup

1. **Host** (e.g., Pico/ESP32) <--(I2C)--> **DS2482**
2. **DS2482** <--(1-Wire)--> **DS28E18**
3. **DS28E18** <--(I2C)--> **SHT4x**

## Installation

### PlatformIO
Add the following to your `platformio.ini`:
```ini
lib_deps =
    https://github.com/DanielSart/SHT40_DS28E18.git
```

### Arduino IDE
1. Download the repository as a ZIP.
2. Install via `Sketch -> Include Library -> Add .ZIP Library...`.
3. Ensure you also have the dependencies installed.

## Usage Example

```cpp
#include <Wire.h>
#include <Adafruit_DS248x.h>
#include "DS28E18.h"
#include "SHT40_DS28E18.h"

// Instances
Adafruit_DS248x ds2482;
DS28E18 ds28e18(ds2482);
SHT40_DS28E18 sht40(ds28e18);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!ds2482.begin(&Wire, 0x18)) {
    Serial.println("DS2482 not found!");
    while (1);
  }

  if (!ds28e18.begin(true)) {
    Serial.println("DS28E18 init failed!");
    while (1);
  }
  
  ds28e18.skipROM();
  ds28e18.initializeGPIO();

  if(!sht40.resetDevice()) {
    Serial.println("SHT40 Soft Reset Failed");
  }
}

void loop() {
  if (sht40.startMeasurement()) {
    delay(20); // Wait for measurement

    float temp, hum;
    if (sht40.readMeasurement(temp, hum)) {
      Serial.print("Temp: "); Serial.print(temp);
      Serial.print(" C, Hum: "); Serial.print(hum); Serial.println(" %");
    }
  }
  delay(2000);
}
```

## License
MIT
