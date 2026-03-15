#ifndef SHT40_DS28E18_H
#define SHT40_DS28E18_H

#include <Arduino.h>
#include "DS28E18.h"
#include "DS28E18_Sequencer.h"
#include "OneWireBus.h"
#include "SequencerBatch.h"

#define SHT4x_DEFAULT_ADDR 0x44 /**< SHT4x I2C Address */
#define SHT4x_NOHEAT_HIGHPRECISION                                             \
  0xFD /**< High precision measurement, no heater */
#define SHT4x_NOHEAT_MEDPRECISION                                              \
  0xF6 /**< Medium precision measurement, no heater */
#define SHT4x_NOHEAT_LOWPRECISION                                              \
  0xE0 /**< Low precision measurement, no heater */

#define SHT4x_HIGHHEAT_1S                                                      \
  0x39 /**< High precision measurement, high heat for 1 sec */
#define SHT4x_HIGHHEAT_100MS                                                   \
  0x32 /**< High precision measurement, high heat for 0.1 sec */
#define SHT4x_MEDHEAT_1S                                                       \
  0x2F /**< High precision measurement, med heat for 1 sec */
#define SHT4x_MEDHEAT_100MS                                                    \
  0x24 /**< High precision measurement, med heat for 0.1 sec */
#define SHT4x_LOWHEAT_1S                                                       \
  0x1E /**< High precision measurement, low heat for 1 sec */
#define SHT4x_LOWHEAT_100MS                                                    \
  0x15 /**< High precision measurement, low heat for 0.1 sec */

#define SHT4x_READSERIAL 0x89 /**< Read Out of Serial Register */
#define SHT4x_SOFTRESET 0x94  /**< Soft Reset */

class SHT40_DS28E18 {
public:
    SHT40_DS28E18(DS28E18 &ds28e18_instance);

    SHT40_DS28E18(OneWireBus &busInstance, uint8_t deviceIndex = 0);

    // Prepares the DS28E18 (no specific SHT40 init needed, but good for checks)
    bool begin();

    // Soft Resets SHT40
    bool resetDevice();

    // Sends command to read serial number
    uint32_t readSerial();

    // Sends command to start measurement
    bool startMeasurement();

    // Reads data after measurement is done
    // Call this >10ms after startMeasurement()
    bool readMeasurement(float &temp, float &hum);

    uint8_t calculateCRC(const uint8_t* data, uint8_t len);

    // ---- Batch mode (SequencerBatch) ----

    // Append a full measurement cycle to a batch:
    // START + WRITE(0xFD) + STOP + DELAY(20ms) + START + READ(6) + STOP
    // This builds a full trigger-wait-read cycle. Repeating this several times
    // in one sequencer run is valid and is how single-device batching works.
    // Returns a handle (>=0) for later result extraction, or -1 on error.
    int8_t addMeasurementToBatch(SequencerBatch &batch);

    // ---- Split Batch mode ----
    // Use this only when the write and read belong to different devices/
    // addresses that can convert in parallel. Do not use it to queue several
    // back-to-back measurements for the same SHT40; the sensor will NACK
    // while the previous conversion is still running.

    // Appends only the measurement trigger command (WRITE 0xFD) to a batch.
    bool addStartToBatch(SequencerBatch &batch);

    // Appends only the read command (READ 6 bytes) to a batch.
    // Returns a handle (>=0) for later result extraction, or -1 on error.
    int8_t addReadToBatch(SequencerBatch &batch);

    // Extract temperature & humidity from a completed batch using the handle.
    // Returns true on success (CRC valid), false on CRC error or invalid handle.
    bool parseBatchMeasurement(const SequencerBatch &batch, int8_t handle,
                               float &temp, float &hum);

    // Device selection helpers (for bus mode)
    void setDeviceIndex(uint8_t index);
    uint8_t getDeviceIndex() const { return deviceIndex; }

private:
    DS28E18 *dsPtr = nullptr;
    OneWireBus *busPtr = nullptr;
    uint8_t deviceIndex = 0;

    DS28E18_Sequencer seq;
    
    // Internal helper to get the active DS28E18 reference
    DS28E18 &activeDS();
};

#endif
