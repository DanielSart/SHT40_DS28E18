#include <Wire.h>
#include <Adafruit_DS248x.h>
#include "OneWireBus.h"
#include "SHT40_DS28E18.h"
#include "SequencerBatch.h"

// --- How many measurements to batch in one sequencer run ---
#define BATCH_COUNT 3

// --- Instances ---
Adafruit_DS248x ds2482;
OneWireBus oneWireBus(ds2482);
SHT40_DS28E18 sht40(oneWireBus, 0);

// (Shared batch objects are now declared locally within loop)

void setup()
{
  DS28E18_Debug = true;
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("--- DS28E18 + SHT40 Batched Measurement Example ---");
  Serial.println();

  if (!ds2482.begin(&Wire, DS248X_ADDRESS))
  {
    Serial.println("DS2482 not found!");
    while (1)
      ;
  }
  Serial.println("DS2482 initialized.");

  while (!ds2482.OneWireReset())
  {
    Serial.println("Failed to do a 1W reset");
    if (ds2482.shortDetected())
    {
      Serial.println("\tShort detected");
    }
    if (!ds2482.presencePulseDetected())
    {
      Serial.println("\tNo presense pulse");
    }
    delay(1000);
  }

  if (!oneWireBus.begin())
  {
    Serial.println("OneWire Bus Init Failed");
    while (1)
      ;
  }

  if (!oneWireBus.initializeROMs())
  {
    Serial.println("OneWire ROM Init Failed");
    while (1)
      ;
  }

  // Search and register devices
  uint8_t rom[8];
  if (ds2482.OneWireSearch(rom))
  {
    Serial.print("Found device: ");
    for (int j = 0; j < 8; j++)
    {
      if (rom[j] < 16)
        Serial.print("0");
      Serial.print(rom[j], HEX);
      Serial.print(" ");
    }
    Serial.println();
    oneWireBus.addDevice(rom);
  }
  else
  {
    Serial.println("No device found!");
    while (1)
      ;
  }

  if (!oneWireBus.resetAllDeviceStatus())
  {
    Serial.println("Reset Device Status failed");
  }

  Serial.println("Resetting SHT40...");
  if (!sht40.resetDevice())
  {
    Serial.println("SHT40 Soft Reset Failed");
  }
  delay(10);

  Serial.println("Reading Serial Number...");
  uint32_t serial = sht40.readSerial();
  Serial.print("SHT40 Serial: 0x");
  Serial.println(serial, HEX);
  Serial.println();

  Serial.print("System Ready. Will batch ");
  Serial.print(BATCH_COUNT);
  Serial.println(" measurements per sequencer run.");
  Serial.println();
}

void loop()
{
  long startTime = millis();

  // =============================================
  // Build one sequencer batch containing full measurement cycles.
  // Each cycle is: trigger -> sequencer delay -> read back 6 bytes.
  // =============================================
  DS28E18 &ds = oneWireBus.device(0);

  int8_t handles[BATCH_COUNT];
  SequencerBatch batch;

  for (int i = 0; i < BATCH_COUNT; i++)
  {
    handles[i] = sht40.addMeasurementToBatch(batch);
    if (handles[i] < 0)
    {
      Serial.print("Failed to add measurement ");
      Serial.println(i);
      return;
    }
  }

  Serial.print("Measurement batch built. Capacity remaining: ");
  Serial.print(batch.remainingCapacity());
  Serial.print(" bytes, actual sequencer delay: ");
  Serial.print(batch.totalDelayMs());
  Serial.println(" ms");

  if (!batch.execute(ds))
  {
    Serial.println("Measurement batch execution FAILED!");
    return;
  }
  Serial.println("Measurement batch executed successfully!");

  long elapsed = millis() - startTime;

  for (int i = 0; i < BATCH_COUNT; i++)
  {
    float temp = 0.0f;
    float hum = 0.0f;

    if (sht40.parseBatchMeasurement(batch, handles[i], temp, hum))
    {
      Serial.print("  Measurement ");
      Serial.print(i);
      Serial.print(": Temp=");
      Serial.print(temp, 2);
      Serial.print(" C  Hum=");
      Serial.print(hum, 2);
      Serial.println(" %");
    }
    else
    {
      Serial.print("  Measurement ");
      Serial.print(i);
      Serial.println(": FAILED (CRC Error)");
    }
  }

  Serial.print("Total time: ");
  Serial.print(elapsed);
  Serial.println(" ms");
  Serial.println();

  delay(2000);
}
