#include <Wire.h>
#include <Adafruit_DS248x.h>
#include "DS28E18.h"
#include "SHT40_DS28E18.h"

// --- Instances ---
Adafruit_DS248x ds2482;
DS28E18 ds28e18(ds2482);
SHT40_DS28E18 sht40(ds28e18);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Turn on debug to see packet flow if needed
  DS28E18_Debug = false; 

  Serial.println("--- DS28E18 + SHT40 Example ---");

  if (!ds2482.begin(&Wire, 0x18)) {
    Serial.println("DS2482 not found!");
    while (1);
  }
  Serial.println("DS2482 initialized.");

  if (!ds28e18.begin(true)) {
    Serial.println("DS28E18 init failed!");
    while (1);
  }
  
  // Configure for single sensor
  ds28e18.skipROM();
  
  // Configure GPIOs (I2C Pullups on DS28E18 side)
  if (!ds28e18.initializeGPIO()) {
    Serial.println("DS28E18 GPIO Init Failed");
    while(1);
  }
  
  // Clear any POR flags
  ds28e18.resetDeviceStatus();

  Serial.println("System Ready. Resetting SHT40...");
  
  if(!sht40.resetDevice()) {
    Serial.println("SHT40 Soft Reset Failed");
  }
  delay(10);

  Serial.println("Reading Serial Number...");
  uint32_t serial = sht40.readSerial();
  Serial.print("SHT40 Serial: 0x");
  Serial.println(serial, HEX);
}

void loop() {
  // --- Step 1: Trigger Measurement ---
  if (sht40.startMeasurement()) {
    
    // SHT40 High Precision takes approx 10ms
    // We wait 20ms to be safe.
    // Note: This is Arduino delay, not sequencer delay.
    delay(20);

    // --- Step 2: Read Data ---
    float temp = 0.0;
    float hum = 0.0;
    
    if (sht40.readMeasurement(temp, hum)) {
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

  delay(2000);
}