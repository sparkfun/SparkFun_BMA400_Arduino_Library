#ifndef __SPARKFUN_BMA400_H__
#define __SPARKFUN_BMA400_H__

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

#include "bma400_api/bma400.h"

// SparkFun's default and secondary I2C addresses
#define BMA400_I2C_ADDRESS_DEFAULT BMA400_I2C_ADDRESS_SDO_LOW    // 0x14
#define BMA400_I2C_ADDRESS_SECONDARY BMA400_I2C_ADDRESS_SDO_HIGH // 0x15

// Generic error code for invalid settings
#define BMA400_E_INVALID_SETTING (BMA400_E_INVALID_CONFIG-1)

// The BMA400's sensor time register increments at 25.6kHz
// Note - the last 3 bits of the sensor time registers will always be 0, so the
// resolution is effectively 3.2kHz
#define BMA400_TICKS_PER_SECOND 25600

// Struct to hold data about the communication interface being used (I2C or SPI)
struct BMA400_InterfaceData
{
    // Communication interface (I2C or SPI)
    bma400_intf interface;

    // I2C settings
    uint8_t i2cAddress;
    TwoWire* i2cPort;

    // SPI settings
    uint8_t spiCSPin;
    uint32_t spiClockFrequency;
};

// Struct to hold acceleration data
struct BMA400_SensorData
{
    // Acceleration in g's
    float x;
    float y;
    float z;

    // Time of this data in milliseconds, measured by sensor
    uint32_t sensorTimeMillis;
};

// Enum for setting individual accel parameters
enum BMA400_AccelParam
{
    BMA400_ODR,
    BMA400_RANGE,
    BMA400_DATA_SRC,
    BMA400_OSR,
    BMA400_OSR_LP,
    BMA400_FILT1_BW,
    BMA400_INT_CHAN
};

class BMA400
{
    public:
        // Constructor
        BMA400();

        // Sensor initialization, must specify communication interface
        int8_t beginI2C(uint8_t address = BMA400_I2C_ADDRESS_DEFAULT, TwoWire& wirePort = Wire);
        int8_t beginSPI(uint8_t csPin, uint32_t clockFrequency = 100000);

        // Configuration control
        int8_t setMode(uint8_t mode);
        int8_t getMode(uint8_t* mode);
        int8_t setRange(uint8_t range);
        int8_t getRange(uint8_t* range);
        int8_t setODR(uint8_t odr);
        int8_t getODR(uint8_t* odr);
        int8_t setOSR(uint8_t osr);
        int8_t getOSR(uint8_t* osr);
        int8_t setOSRLP(uint8_t osrLP);
        int8_t getOSRLP(uint8_t* osrLP);
        int8_t setDataSource(uint8_t source);
        int8_t getDataSource(uint8_t* source);
        int8_t setFilter1Bandwidth(uint8_t bw);
        int8_t getFilter1Bandwidth(uint8_t* bw);
        int8_t getStepCount(uint32_t* count, uint8_t* activityType);
        int8_t selfTest();

        // Data acquisistion
        int8_t getSensorData(BMA400_SensorData* data, bool sensorTime = false);
        int8_t getTemperature(float* temp);

        // Interrupt control
        int8_t setInterruptPinMode(bma400_int_chan channel, uint8_t mode);
        int8_t enableInterrupt(bma400_int_type intType, bool enable);
        int8_t getInterruptStatus(uint16_t* status);

        // Interrupt features
        int8_t setDRDYInterruptChannel(bma400_int_chan channel);
        int8_t setGeneric1Interrupt(bma400_gen_int_conf* config);
        int8_t setGeneric2Interrupt(bma400_gen_int_conf* config);
        int8_t setOrientationChangeInterrupt(bma400_orient_int_conf* config);
        int8_t setTapInterrupt(bma400_tap_conf* config);
        int8_t setStepCounterInterrupt(bma400_step_int_conf* config);
        int8_t setActivityChangeInterrupt(bma400_act_ch_conf* config);

        // FIFO control
        int8_t setFIFOConfigFlags(uint8_t flags);
        int8_t setFIFOWatermark(uint16_t numData);
        int8_t setFIFOFullInterruptChannel(bma400_int_chan channel);
        int8_t setFIFOWatermarkInterruptChannel(bma400_int_chan channel);
        int8_t getFIFOLength(uint16_t* numData);
        int8_t getFIFOData(BMA400_SensorData* data, uint16_t* numData);
        int8_t flushFIFO();

    private:
        // Sensor initialization, after communication interface has been selected
        int8_t begin();

        // When calling bma400_get_sensor_conf(), the BMA400_ACCEL category has
        // several unrelated parameters that all get set at once. These are
        // generic functions to only get/set one parameter at a time
        int8_t setAccelParam(BMA400_AccelParam param, uint8_t val);
        int8_t getAccelParam(BMA400_AccelParam param, uint8_t* val);

        // Convert from raw data (bma400_sensor_data) to g's (BMA400_SensorData)
        void convertRawData(bma400_sensor_data* rawData, BMA400_SensorData* data, uint8_t range, bool bitWidth = 12);

        uint16_t numFIFODataToBytes(uint8_t fifoFlags, uint16_t numData);

        // Read/write helper functions
        static BMA400_INTF_RET_TYPE readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr);
        static BMA400_INTF_RET_TYPE writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr);

        // Deley helper function
        static void usDelay(uint32_t period, void* interfacePtr);

        // Reference to the sensor
        struct bma400_dev sensor;

        // Information about the selected communication interface (I2C or SPI)
        BMA400_InterfaceData interfaceData;
};

#endif