#include "SHT40_DS28E18.h"

SHT40_DS28E18::SHT40_DS28E18(DS28E18 &ds28e18_instance)
    : dsPtr(&ds28e18_instance), busPtr(nullptr), deviceIndex(0) {}

SHT40_DS28E18::SHT40_DS28E18(OneWireBus &busInstance, uint8_t index)
    : dsPtr(nullptr), busPtr(&busInstance), deviceIndex(index) {}

bool SHT40_DS28E18::begin() {
    return true; // placeholder, no sensor init required
}

DS28E18 &SHT40_DS28E18::activeDS() {
    if (busPtr) {
        // Use the bus’s DS28E18 handle for indexed device
        return busPtr->device(deviceIndex);
    }
    // fallback to single DS28E18 reference
    return *dsPtr;
}

void SHT40_DS28E18::setDeviceIndex(uint8_t index) {
    deviceIndex = index;
}

bool SHT40_DS28E18::resetDevice() {
    DS28E18 &ds = activeDS();
    seq.clear();
    seq.addStart();
    seq.addWriteByte(SHT4x_DEFAULT_ADDR, SHT4x_SOFTRESET);
    seq.addStop();
    seq.addDelay(0x00); // 1ms delay for reset

    // 1. Write packets to DS28E18 SRAM
    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) {
        return false;
    }

    // 2. Execute packets
    uint8_t result;
    if (!ds.runSequencer(0, seq.getLength(), result)) {
        return false;
    }
    
    return true;
}

uint32_t SHT40_DS28E18::readSerial(void) {
    DS28E18 &ds = activeDS();
    seq.clear();
    seq.addStart();
    // 1. Send Command
    seq.addWriteByte(SHT4x_DEFAULT_ADDR, SHT4x_READSERIAL);
    // 2. Send Stop
    seq.addStop();
    // 2. Delay (SHT40 needs ~1ms before read)
    seq.addDelay(0x00); 
    // 3. Start + Address Read + Read 6 bytes
    seq.addStart();
    seq.addRead(SHT4x_DEFAULT_ADDR, 6);
    seq.addStop();

    // Write Sequencer
    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return 0;

    // Run Sequencer
    uint8_t result;
    if (!ds.runSequencer(0, seq.getLength(), result)) return 0;

    // Calculate Offset:
    // [0] Result
    // [1] Start
    // [2-6] Write Pkt (0xE3, Len, Addr, Cmd, Stop)
    // [7-8] Delay (0xDD, 0x00)
    // [9] Start
    // [10-12] Write Addr Read (0xE3, 0x01, Addr|1)
    // [13-14] Read Op (0xD3, Len)
    // [15-19] DATA (6 bytes) <--- Offset is 15
    uint16_t dataOffset = 15; 
    
    uint8_t rawData[32]; 
    uint16_t readLen = 0;

    // Read back SRAM to get data
    if (!ds.readSequencer(0, rawData, seq.getLength(), readLen)) return 0;

    uint8_t *rx = &rawData[dataOffset];

    // CRC Checks
    if (calculateCRC(rx, 2) != rx[2]) {
        Serial.print("Data1: ");
        Serial.print(rx[0], HEX);
        Serial.print(", Data2: ");
        Serial.print(rx[1], HEX);
        Serial.print(", CRC (device): ");
        Serial.print(rx[2], HEX);
        Serial.print(", CRC (calc): ");
        Serial.println(calculateCRC(rx, 2), HEX);
        Serial.println("SHT40 CRC Error");
        return false;
    }
    if (calculateCRC(&rx[3], 2) != rx[5]) {
        Serial.print("Data1: ");
        Serial.print(rx[3], HEX);
        Serial.print(", Data2: ");
        Serial.print(rx[4], HEX);
        Serial.print(", CRC (device): ");
        Serial.print(rx[5], HEX);
        Serial.print(", CRC (calc): ");
        Serial.println(calculateCRC(&rx[3], 2), HEX);
        Serial.println("SHT40 CRC Error");
        return false;
    }

    uint32_t serial = 0;
    serial |= ((uint32_t)rawData[dataOffset + 0]) << 24;
    serial |= ((uint32_t)rawData[dataOffset + 1]) << 16;
    serial |= ((uint32_t)rawData[dataOffset + 3]) << 8;  // Skip CRC at +2
    serial |= ((uint32_t)rawData[dataOffset + 4]);       // Skip CRC at +5

    return serial;
}

bool SHT40_DS28E18::startMeasurement() {
    DS28E18 &ds = activeDS();
    seq.clear();
    seq.addStart();
    seq.addWriteByte(SHT4x_DEFAULT_ADDR, SHT4x_NOHEAT_HIGHPRECISION);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return false;

    uint8_t result;
    if (!ds.runSequencer(0, seq.getLength(), result)) return false;

    return true;
}

bool SHT40_DS28E18::readMeasurement(float &temp, float &hum) {
    DS28E18 &ds = activeDS();
    seq.clear();
    seq.addStart();
    // Standard I2C Read: Start -> Addr+Read -> Data
    seq.addRead(SHT4x_DEFAULT_ADDR, 6);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return false;

    uint8_t result;
    if (!ds.runSequencer(0, seq.getLength(), result)) return false;

    // Calculate Offset:
    // [0] Result (0xAA)
    // [1] Start (0x02)
    // [2-4] Write Addr Read (0xE3, 0x01, Addr|1)
    // [5-6] Read Op (0xD3, 0x06)
    // [7-12] DATA (6 bytes) <--- Offset is 7
    
    uint16_t dataOffset = 7; 
    uint8_t rawData[20];
    uint16_t readLen = 0;

    if (!ds.readSequencer(0, rawData, seq.getLength(), readLen)) return false;

    uint8_t *rx = &rawData[dataOffset];

    // CRC Checks
    if (calculateCRC(rx, 2) != rx[2]) {
        Serial.print("Data1: ");
        Serial.print(rx[0], HEX);
        Serial.print(", Data2: ");
        Serial.print(rx[1], HEX);
        Serial.print(", CRC (device): ");
        Serial.print(rx[2], HEX);
        Serial.print(", CRC (calc): ");
        Serial.println(calculateCRC(rx, 2), HEX);
        Serial.println("SHT40 CRC Error");
        return false;
    }
    if (calculateCRC(&rx[3], 2) != rx[5]) {
        Serial.print("Data1: ");
        Serial.print(rx[3], HEX);
        Serial.print(", Data2: ");
        Serial.print(rx[4], HEX);
        Serial.print(", CRC (device): ");
        Serial.print(rx[5], HEX);
        Serial.print(", CRC (calc): ");
        Serial.println(calculateCRC(&rx[3], 2), HEX);
        Serial.println("SHT40 CRC Error");
        return false;
    }

    // Conversion
    uint16_t t_ticks = (rx[0] << 8) | rx[1];
    temp = -45.0f + 175.0f * ((float)t_ticks / 65535.0f);

    uint16_t rh_ticks = (rx[3] << 8) | rx[4];
    hum = -6.0f + 125.0f * ((float)rh_ticks / 65535.0f);
    if (hum > 100.0f) hum = 100.0f;
    if (hum < 0.0f) hum = 0.0f;

    return true;
}
// -----------------------------------------------------------------------
// Batch mode: addMeasurementToBatch
// Appends a full measurement cycle to a SequencerBatch:
//   START + WRITE(0xFD) + STOP + DELAY(20ms) + START + READ(6) + STOP
// -----------------------------------------------------------------------
int8_t SHT40_DS28E18::addMeasurementToBatch(SequencerBatch &batch) {
    uint8_t cmd = SHT4x_NOHEAT_HIGHPRECISION;
    // addI2CWriteRead: write cmd, wait 20ms, then read 6 bytes
    return batch.addI2CWriteRead(SHT4x_DEFAULT_ADDR, &cmd, 1, 6, 20);
}

// -----------------------------------------------------------------------
// Split Batch mode: addStartToBatch
// Appends just the measurement trigger command (WRITE 0xFD).
// -----------------------------------------------------------------------
bool SHT40_DS28E18::addStartToBatch(SequencerBatch &batch) {
    uint8_t cmd = SHT4x_NOHEAT_HIGHPRECISION;
    return batch.addI2CWrite(SHT4x_DEFAULT_ADDR, &cmd, 1);
}

// -----------------------------------------------------------------------
// Split Batch mode: addReadToBatch
// Appends just the read command (READ 6 bytes).
// -----------------------------------------------------------------------
int8_t SHT40_DS28E18::addReadToBatch(SequencerBatch &batch) {
    return batch.addI2CRead(SHT4x_DEFAULT_ADDR, 6);
}

// -----------------------------------------------------------------------
// Batch mode: parseBatchMeasurement
// Extracts temperature & humidity from a completed batch.
// -----------------------------------------------------------------------
bool SHT40_DS28E18::parseBatchMeasurement(const SequencerBatch &batch,
                                           int8_t handle,
                                           float &temp, float &hum)
{
    const uint8_t *rx = batch.getReadPtr(handle);
    if (!rx) {
        Serial.println("SHT40 batch: invalid handle or batch not executed");
        return false;
    }

    // rx[0..1] = temp data, rx[2] = temp CRC
    // rx[3..4] = hum data,  rx[5] = hum CRC
    if (calculateCRC(rx, 2) != rx[2]) {
        Serial.print("Data1: "); Serial.print(rx[0], HEX);
        Serial.print(", Data2: "); Serial.print(rx[1], HEX);
        Serial.print(", CRC (device): "); Serial.print(rx[2], HEX);
        Serial.print(", CRC (calc): "); Serial.println(calculateCRC(rx, 2), HEX);
        Serial.println("SHT40 Batch CRC Error (temp)");
        return false;
    }
    if (calculateCRC(&rx[3], 2) != rx[5]) {
        Serial.print("Data1: "); Serial.print(rx[3], HEX);
        Serial.print(", Data2: "); Serial.print(rx[4], HEX);
        Serial.print(", CRC (device): "); Serial.print(rx[5], HEX);
        Serial.print(", CRC (calc): "); Serial.println(calculateCRC(&rx[3], 2), HEX);
        Serial.println("SHT40 Batch CRC Error (hum)");
        return false;
    }

    // Convert
    uint16_t t_ticks = (rx[0] << 8) | rx[1];
    temp = -45.0f + 175.0f * ((float)t_ticks / 65535.0f);

    uint16_t rh_ticks = (rx[3] << 8) | rx[4];
    hum = -6.0f + 125.0f * ((float)rh_ticks / 65535.0f);
    if (hum > 100.0f) hum = 100.0f;
    if (hum < 0.0f) hum = 0.0f;

    return true;
}

uint8_t SHT40_DS28E18::calculateCRC(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc = (crc << 1);
        }
    }
    return crc;
}