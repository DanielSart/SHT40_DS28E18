#include <Wire.h>
#include <Adafruit_DS248x.h>
#include "OneWireBus.h"
#include "SHT40_DS28E18.h"

#define NUMBER_OF_DS28E18 2

// --- Instances ---
Adafruit_DS248x ds2482;
OneWireBus oneWireBus(ds2482);
SHT40_DS28E18 sht40_0(oneWireBus, 0);
SHT40_DS28E18 sht40_1(oneWireBus, 1);

long lastTime;
long elapsedTime;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  //oneWireBus.enableDebug();

  Serial.println("--- DS28E18 + Dual SHT40 Example ---");

  if (!ds2482.begin(&Wire, DS248X_ADDRESS)) {
    Serial.println("DS2482 not found!");
    while (1);
  }
  Serial.println("DS2482 initialized.");

  while (!ds2482.OneWireReset()) {
    Serial.println("Failed to do a 1W reset");
    if (ds2482.shortDetected()) {
      Serial.println("\tShort detected");
    }
    if (!ds2482.presencePulseDetected()) {
      Serial.println("\tNo presense pulse");
    }
    delay(1000);
  }

  if (!oneWireBus.begin()) {
    Serial.println("OneWire Bus Init Failed");
    while(1);
  }

  if (!oneWireBus.initializeROMs()) {
    Serial.println("OneWire ROM Init Failed");
    while(1);
  }

  for(int i = 0; i < NUMBER_OF_DS28E18; i++){
    uint8_t rom[8];
    Serial.print(i);
    Serial.print(": ");
    if (ds2482.OneWireSearch(rom)) {
      for (int j = 0; j < 8; j++) {
        if (rom[j] < 16) {
          Serial.print("0");
        }
        Serial.print(rom[j], HEX);
        Serial.print(" ");
      }
    } else {
      Serial.print("No Device found!");
    }
    oneWireBus.addDevice(rom);
    Serial.println();
  }

  //oneWireBus.enableOverdrive();

  if (!oneWireBus.resetAllDeviceStatus()) {
    Serial.println("Reset Device Status failed");
  }

  Serial.println("System Ready. Resetting SHT40...");
  
  lastTime = millis();

  if(!sht40_0.resetDevice()) {
    Serial.println("SHT40 Soft Reset Failed");
  }
  if(!sht40_1.resetDevice()) {
    Serial.println("SHT40 Soft Reset Failed");
  }
  delay(10);

  Serial.println("Reading Serial Number...");
  uint32_t serial = sht40_0.readSerial();
  Serial.print("SHT40 Serial: 0x");
  Serial.println(serial, HEX);

  Serial.println("Reading Serial Number...");
  serial = sht40_1.readSerial();
  Serial.print("SHT40 Serial: 0x");
  Serial.println(serial, HEX);

  elapsedTime = millis() - lastTime;
  Serial.print("t = ");
  Serial.print(elapsedTime);
  Serial.println(" ms");
  Serial.println();
}

void loop() {
  // --- Step 1: Trigger Measurement ---
  
  lastTime = millis();

  if (sht40_0.startMeasurement()) {
    
    // SHT40 High Precision takes approx 10ms
    // We wait 20ms to be safe.
    // Note: This is Arduino delay, not sequencer delay.
    delay(20);

    // --- Step 2: Read Data ---
    float temp = 0.0;
    float hum = 0.0;
    
    if (sht40_0.readMeasurement(temp, hum)) {
      Serial.println("Sensor 0: ");
      Serial.print("Temp: ");
      Serial.print(temp);
      Serial.print(" C \t Humidity: ");
      Serial.print(hum);
      Serial.println(" %");
    } else {
      Serial.println("Read Failed (CRC or NACK)");
    }
    
  } else {
    Serial.println("Start Measurement Failed");
  }


  if (sht40_1.startMeasurement()) {
    
    // SHT40 High Precision takes approx 10ms
    // We wait 20ms to be safe.
    // Note: This is Arduino delay, not sequencer delay.
    delay(20);

    // --- Step 2: Read Data ---
    float temp = 0.0;
    float hum = 0.0;
    
    if (sht40_1.readMeasurement(temp, hum)) {
      Serial.println("Sensor 1: ");
      Serial.print("Temp: ");
      Serial.print(temp);
      Serial.print(" C \t Humidity: ");
      Serial.print(hum);
      Serial.println(" %");
    } else {
      Serial.println("Read Failed (CRC or NACK)");
    }
    
  } else {
    Serial.println("Start Measurement Failed");
  }
  elapsedTime = millis() - lastTime;
  Serial.print("t = ");
  Serial.print(elapsedTime);
  Serial.println(" ms");

  Serial.println();

  while(1);
  
  delay(2000);
}