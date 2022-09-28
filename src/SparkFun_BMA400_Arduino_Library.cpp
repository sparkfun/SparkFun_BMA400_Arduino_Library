#include "SparkFun_BMA400_Arduino_Library.h"

BMA400::BMA400()
{
    // Nothing to do
}

int8_t BMA400::beginI2C(uint8_t address, TwoWire& wirePort)
{
    // Check whether address is valid option
    if(address != BMA400_I2C_ADDRESS_DEFAULT && address != BMA400_I2C_ADDRESS_SECONDARY)
    {
        // Invalid option, don't do anything
        return BMA400_E_INVALID_SETTING;
    }

    // Address is valid option
    interfaceData.i2cAddress = address;
    interfaceData.i2cPort = &wirePort;

    // Set interface
    sensor.intf = BMA400_I2C_INTF;
    interfaceData.interface = BMA400_I2C_INTF;

    // Initialize sensor
    return begin();
}

int8_t BMA400::beginSPI(uint8_t csPin, uint32_t clockFrequency)
{
    // Set up chip select pin
    interfaceData.spiCSPin = csPin;
    digitalWrite(csPin, HIGH); // Write high now to ensure pin doesn't go low
    pinMode(csPin, OUTPUT);

    // Set desired clock frequency
    interfaceData.spiClockFrequency = clockFrequency;

    // Set interface
    sensor.intf = BMA400_SPI_INTF;
    interfaceData.interface = BMA400_SPI_INTF;

    // Initialize sensor
    return begin();
}

int8_t BMA400::begin()
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Set helper function pointers
    sensor.read = readRegisters;
    sensor.write = writeRegisters;
    sensor.delay_us = usDelay;
    sensor.intf_ptr = &interfaceData;

    // Reset the sensor
    err = bma400_soft_reset(&sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Initialize the sensor
    err = bma400_init(&sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Set to normal power mode
    return setMode(BMA400_MODE_NORMAL);
}

int8_t BMA400::setMode(uint8_t mode)
{
    return bma400_set_power_mode(mode, &sensor);
}

int8_t BMA400::getMode(uint8_t* mode)
{
    return bma400_get_power_mode(mode, &sensor);
}

int8_t BMA400::setAutoWakeup(bma400_auto_wakeup_conf* config)
{
    bma400_device_conf deviceConfig =
    {
        .type = BMA400_AUTOWAKEUP_TIMEOUT,
        .param = {.auto_wakeup = *config}
    };
    return bma400_set_device_conf(&deviceConfig, 1, &sensor);
}

int8_t BMA400::setAutoLowPower(bma400_auto_lp_conf* config)
{
    bma400_device_conf deviceConfig =
    {
        .type = BMA400_AUTO_LOW_POWER,
        .param = {.auto_lp = *config}
    };
    return bma400_set_device_conf(&deviceConfig, 1, &sensor);
}

int8_t BMA400::setAccelParam(BMA400_AccelParam param, uint8_t val)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current accel config so we don't accidentally change anything
    bma400_sensor_conf config =
    {
        .type = BMA400_ACCEL,
        .param = {0}
    };
    err = bma400_get_sensor_conf(&config, 1, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Change requested parameter to new value if it's valid
    switch(param)
    {
        case BMA400_ODR:
            if(val < BMA400_ODR_12_5HZ || val > BMA400_ODR_800HZ) return BMA400_E_INVALID_SETTING;
            config.param.accel.odr = val;
            break;
        case BMA400_RANGE:
            if(val > BMA400_RANGE_16G) return BMA400_E_INVALID_SETTING;
            config.param.accel.range = val;
            break;
        case BMA400_DATA_SRC:
            if(val > BMA400_DATA_SRC_ACCEL_FILT_LP) return BMA400_E_INVALID_SETTING;
            config.param.accel.data_src = val;
            break;
        case BMA400_OSR:
            if(val > BMA400_ACCEL_OSR_SETTING_3) return BMA400_E_INVALID_SETTING;
            config.param.accel.osr = val;
            break;
        case BMA400_OSR_LP:
            if(val > BMA400_ACCEL_OSR_SETTING_3) return BMA400_E_INVALID_SETTING;
            config.param.accel.osr_lp = val;
            break;
        case BMA400_FILT1_BW:
            if(val > BMA400_ACCEL_FILT1_BW_1) return BMA400_E_INVALID_SETTING;
            config.param.accel.filt1_bw = val;
            break;
        case BMA400_INT_CHAN:
            if(val > BMA400_MAP_BOTH_INT_PINS) return BMA400_E_INVALID_SETTING;
            config.param.accel.int_chan = (bma400_int_chan) val;
            break;
        default:
            return BMA400_E_INVALID_SETTING;
            break;
    }

    // Set new config
    return bma400_set_sensor_conf(&config, 1, &sensor);
}

int8_t BMA400::getAccelParam(BMA400_AccelParam param, uint8_t* val)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current accel config
    bma400_sensor_conf config =
    {
        .type = BMA400_ACCEL,
        .param = {0}
    };
    err = bma400_get_sensor_conf(&config, 1, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Get requested parameter
    switch(param)
    {
        case BMA400_ODR:
            *val = config.param.accel.odr;
            break;
        case BMA400_RANGE:
            *val = config.param.accel.range;
            break;
        case BMA400_DATA_SRC:
            *val = config.param.accel.data_src;
            break;
        case BMA400_OSR:
            *val = config.param.accel.osr;
            break;
        case BMA400_OSR_LP:
            *val = config.param.accel.osr_lp;
            break;
        case BMA400_FILT1_BW:
            *val = config.param.accel.filt1_bw;
            break;
        case BMA400_INT_CHAN:
            *val = config.param.accel.int_chan;
            break;
        default:
            return BMA400_E_INVALID_SETTING;
            break;
    }

    return BMA400_OK;
}

int8_t BMA400::setRange(uint8_t range)
{
    return setAccelParam(BMA400_RANGE, range);
}

int8_t BMA400::getRange(uint8_t* range)
{
    return getAccelParam(BMA400_RANGE, range);
}

int8_t BMA400::setODR(uint8_t odr)
{
    return setAccelParam(BMA400_ODR, odr);
}

int8_t BMA400::getODR(uint8_t* odr)
{
    return getAccelParam(BMA400_ODR, odr);
}

int8_t BMA400::setOSR(uint8_t osr)
{
    return setAccelParam(BMA400_OSR, osr);
}

int8_t BMA400::getOSR(uint8_t* osr)
{
    return getAccelParam(BMA400_OSR, osr);
}

int8_t BMA400::setOSRLP(uint8_t osrLP)
{
    return setAccelParam(BMA400_OSR_LP, osrLP);
}

int8_t BMA400::getOSRLP(uint8_t* osrLP)
{
    return getAccelParam(BMA400_OSR_LP, osrLP);
}

int8_t BMA400::setDataSource(uint8_t source)
{
    return setAccelParam(BMA400_DATA_SRC, source);
}

int8_t BMA400::getDataSource(uint8_t* source)
{
    return getAccelParam(BMA400_DATA_SRC, source);
}

int8_t BMA400::setFilter1Bandwidth(uint8_t bw)
{
    return setAccelParam(BMA400_FILT1_BW, bw);
}

int8_t BMA400::getFilter1Bandwidth(uint8_t* bw)
{
    return getAccelParam(BMA400_FILT1_BW, bw);
}

int8_t BMA400::getStepCount(uint32_t* count, uint8_t* activityType)
{
    return bma400_get_steps_counted(count, activityType, &sensor);
}

int8_t BMA400::selfTest()
{
    return bma400_perform_self_test(&sensor);
}

int8_t BMA400::getSensorData(BMA400_SensorData* data, bool sensorTime)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Set whether to include sensor time of this measurement
    uint8_t dataSel = sensorTime ? BMA400_DATA_SENSOR_TIME : BMA400_DATA_ONLY;

    // Get raw data from sensor
    bma400_sensor_data rawData = {0};
    err = bma400_get_accel_data(dataSel, &rawData, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Get current measurement range setting
    uint8_t range = 0;
    err = getRange(&range);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Convert raw data to g's
    convertRawData(&rawData, data, range);

    return BMA400_OK;
}

void BMA400::convertRawData(bma400_sensor_data* rawData, BMA400_SensorData* data, uint8_t range, uint8_t bitWidth)
{
    // Convert range setting to g-range. This computation is shorthand for the
    // following settings:
    // 
    // rangeSetting     | gRange
    // BMA400_RANGE_2G  | 2
    // BMA400_RANGE_4G  | 4
    // BMA400_RANGE_8G  | 8
    // BMA400_RANGE_16G | 16
    uint8_t gRange = 2 << range;

    // Convert xyz data from raw to g's. Raw data are 12/8-bit signed integers,
    // where the maximum raw value corresponds to the max of the range setting
    float rawToGs = gRange / pow(2, (bitWidth-1));
    data->x = rawData->x * rawToGs;
    data->y = rawData->y * rawToGs;
    data->z = rawData->z * rawToGs;

    // Convert raw sensor time to milliseconds
    data->sensorTimeMillis = rawData->sensortime * 1000 / BMA400_TICKS_PER_SECOND;
}

int8_t BMA400::getTemperature(float* temp)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get raw data from sensor
    int16_t rawTemp = 0;
    err = bma400_get_temperature_data(&rawTemp, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Convert raw data to degrees C
    *temp = rawTemp / 10.0;
    return BMA400_OK;
}

int8_t BMA400::setInterruptPinMode(bma400_int_chan channel, uint8_t mode)
{
    bma400_device_conf config =
    {
        .type = BMA400_INT_PIN_CONF,
        .param = {.int_conf =
        {
            .int_chan = channel,
            .pin_conf = mode
        }}
    };
    return bma400_set_device_conf(&config, 1, &sensor);
}

int8_t BMA400::enableInterrupt(bma400_int_type intType, bool enable)
{
    bma400_int_enable config =
    {
        .type = intType,
        .conf = enable ? BMA400_ENABLE : BMA400_DISABLE
    };
    return bma400_enable_interrupt(&config, 1, &sensor);
}

int8_t BMA400::getInterruptStatus(uint16_t* status)
{
    return bma400_get_interrupt_status(status, &sensor);
}

int8_t BMA400::setDRDYInterruptChannel(bma400_int_chan channel)
{
    return setAccelParam(BMA400_INT_CHAN, channel);
}

int8_t BMA400::setGeneric1Interrupt(bma400_gen_int_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_GEN1_INT,
        .param = {.gen_int = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

int8_t BMA400::setGeneric2Interrupt(bma400_gen_int_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_GEN2_INT,
        .param = {.gen_int = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

int8_t BMA400::setOrientationChangeInterrupt(bma400_orient_int_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_ORIENT_CHANGE_INT,
        .param = {.orient = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

int8_t BMA400::setTapInterrupt(bma400_tap_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_TAP_INT,
        .param = {.tap = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

int8_t BMA400::setStepCounterInterrupt(bma400_step_int_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_STEP_COUNTER_INT,
        .param {.step_cnt = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

int8_t BMA400::setActivityChangeInterrupt(bma400_act_ch_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_ACTIVITY_CHANGE_INT,
        .param = {.act_ch = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

int8_t BMA400::setWakeupInterrupt(bma400_wakeup_conf* config)
{
    bma400_device_conf deviceConfig =
    {
        .type = BMA400_AUTOWAKEUP_INT,
        .param = {.wakeup = *config}
    };
    return bma400_set_device_conf(&deviceConfig, 1, &sensor);
}

int8_t BMA400::setFIFOConfigFlags(uint8_t flags)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current FIFO config
    struct bma400_device_conf config =
    {
        .type = BMA400_FIFO_CONF,
        .param = {0}
    };
    err = bma400_get_device_conf(&config, 1, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Change config flags as requested
    config.param.fifo_conf.conf_regs = flags;
    config.param.fifo_conf.conf_status = BMA400_ENABLE; // This forces flags to be overwritten
    return bma400_set_device_conf(&config, 1, &sensor);
}

int8_t BMA400::setFIFOWatermark(uint16_t numData)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current FIFO config
    struct bma400_device_conf config =
    {
        .type = BMA400_FIFO_CONF,
        .param = {0}
    };
    err = bma400_get_device_conf(&config, 1, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Compute the total number of bytes for this watermark level
    uint16_t watermarkBytes = numFIFODataToBytes(config.param.fifo_conf.conf_regs, numData);

    // Check whether this exceeds the FIFO buffer's size (1KB)
    if(watermarkBytes > 1024)
    {
        // Too many bytes, can't se watermark level
        return BMA400_E_INVALID_SETTING;
    }

    // Set watermark
    config.param.fifo_conf.fifo_watermark = watermarkBytes;
    return bma400_set_device_conf(&config, 1, &sensor);
}

int8_t BMA400::setFIFOFullInterruptChannel(bma400_int_chan channel)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current FIFO config
    struct bma400_device_conf config =
    {
        .type = BMA400_FIFO_CONF,
        .param = {0}
    };
    err = bma400_get_device_conf(&config, 1, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Change config flags as requested
    config.param.fifo_conf.fifo_full_channel = channel;
    return bma400_set_device_conf(&config, 1, &sensor);
}

int8_t BMA400::setFIFOWatermarkInterruptChannel(bma400_int_chan channel)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current FIFO config
    struct bma400_device_conf config =
    {
        .type = BMA400_FIFO_CONF,
        .param = {0}
    };
    err = bma400_get_device_conf(&config, 1, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Change config flags as requested
    config.param.fifo_conf.fifo_full_channel = channel;
    return bma400_set_device_conf(&config, 1, &sensor);
}

int8_t BMA400::getFIFOLength(uint16_t* numData)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // None of the functions available in bma400.h enable getting the current
    // FIFO length, so the regsiters have to be read manually
    uint8_t data[2] = {0};
    err = bma400_get_regs(BMA400_REG_FIFO_LENGTH, data, 2, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    *numData = ((uint16_t) data[1] << 8) | data[0];

    return BMA400_OK;
}

int8_t BMA400::getFIFOData(BMA400_SensorData* data, uint16_t* numData)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current FIFO config
    struct bma400_device_conf config =
    {
        .type = BMA400_FIFO_CONF,
        .param = {0}
    };
    err = bma400_get_device_conf(&config, 1, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Short variable name to reference FIFO config flags
    uint8_t flags = config.param.fifo_conf.conf_regs;

    // Compute number of bytes that need to be read out from FIFO
    uint16_t numBytes = numFIFODataToBytes(flags, *numData);

    // Determine whether sensor time is enabled
    if((flags & BMA400_FIFO_TIME_EN) != 0)
    {
        // Sensor time is enabled, so one extra time frame will be output after
        // all data frames are read. If the buffer already has lots of data,
        // it's possible extra data frames will be added to the FIFO before we
        // reach the end, especially at high ODR. Here we add some extra bytes
        // to account for the extra data frames, plus the one sensor frame
        numBytes += BMA400_FIFO_BYTES_OVERREAD;
    }

    // Create a byte buffer to store the raw FIFO bytes
    uint8_t byteBuffer[numBytes];

    // Get raw data from FIFO buffer
    bma400_fifo_data fifoData = {0};
    fifoData.data = byteBuffer;
    fifoData.length = numBytes;
    err = bma400_get_fifo_data(&fifoData, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Create array of raw data
    bma400_sensor_data rawData[*numData];

    // Extract sensor data out of FIFO data
    err = bma400_extract_accel(&fifoData, rawData, numData, &sensor);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Determine the number of bits per data
    uint8_t bitWidth = ((flags & BMA400_FIFO_8_BIT_EN) != 0) ? 8 : 12;

    // Get current measurement range setting
    uint8_t range = 0;
    err = getRange(&range);
    if(err != BMA400_OK)
    {
        return err;
    }

    // Convert raw data to g's for each data frame
    for(uint16_t i = 0; i < *numData; i++)
    {
        // Add sensor time to each data frame before converting
        rawData[i].sensortime = fifoData.fifo_sensor_time;
        convertRawData(&rawData[i], &data[i], range, bitWidth);
    }

    return BMA400_OK;
}

int8_t BMA400::flushFIFO()
{
    return bma400_set_fifo_flush(&sensor);
}

uint16_t BMA400::numFIFODataToBytes(uint8_t fifoFlags, uint16_t numData)
{
    // Determine how many axes are being stored in the FIFO buffer
    uint8_t numAxes = ((fifoFlags & BMA400_FIFO_X_EN) != 0)
                    + ((fifoFlags & BMA400_FIFO_Y_EN) != 0)
                    + ((fifoFlags & BMA400_FIFO_Z_EN) != 0);
    
    // Determine the number of bytes per axis (2 full bytes for 12-bit mode)
    uint8_t bytesPerAxis = ((fifoFlags & BMA400_FIFO_8_BIT_EN) != 0) ? 1 : 2;

    // Compute the total number of bytes per data frame. Data frames include 1
    // header byte, plus 1 or 2 bytes per axis
    uint8_t bytesPerMeasurement = 1 + (numAxes * bytesPerAxis);

    // Compute total number fo bytes for all data frames
    return bytesPerMeasurement * numData;
}

BMA400_INTF_RET_TYPE BMA400::readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr)
{
    // Make sure the number of bytes is valid
    if(numBytes == 0)
    {
        return BMA400_E_COM_FAIL;
    }

    // Get interface data
    BMA400_InterfaceData* interfaceData = (BMA400_InterfaceData*) interfacePtr;

    switch(interfaceData->interface)
    {
        case BMA400_I2C_INTF:
            // Jump to desired register address
            interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);
            interfaceData->i2cPort->write(regAddress);
            if(interfaceData->i2cPort->endTransmission())
            {
                return BMA400_E_COM_FAIL;
            }

            // Read bytes from these registers
            interfaceData->i2cPort->requestFrom(interfaceData->i2cAddress, numBytes);

            // Store all requested bytes
            for(uint32_t i = 0; i < numBytes && interfaceData->i2cPort->available(); i++)
            {
                dataBuffer[i] = interfaceData->i2cPort->read();
            }
            break;

        case BMA400_SPI_INTF:
            // Start transmission
            SPI.beginTransaction(SPISettings(interfaceData->spiClockFrequency, MSBFIRST, SPI_MODE0));
            digitalWrite(interfaceData->spiCSPin, LOW);
            SPI.transfer(regAddress | 0x80);

            // Read all requested bytes
            for(uint32_t i = 0; i < numBytes; i++)
            {
                dataBuffer[i] = SPI.transfer(0);;
            }

            // End transmission
            digitalWrite(interfaceData->spiCSPin, HIGH);
            SPI.endTransaction();
            break;
    }

    return BMA400_OK;
}

BMA400_INTF_RET_TYPE BMA400::writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr)
{
    // Make sure the number of bytes is valid
    if(numBytes == 0)
    {
        return BMA400_E_COM_FAIL;
    }
    // Get interface data
    BMA400_InterfaceData* interfaceData = (BMA400_InterfaceData*) interfacePtr;

    // Determine which interface we're using
    switch(interfaceData->interface)
    {
        case BMA400_I2C_INTF:
            // Begin transmission
            interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);

            // Write the address
            interfaceData->i2cPort->write(regAddress);
            
            // Write all the data
            for(uint32_t i = 0; i < numBytes; i++)
            {
                interfaceData->i2cPort->write(dataBuffer[i]);
            }

            // End transmission
            if(interfaceData->i2cPort->endTransmission())
            {
                return BMA400_E_COM_FAIL;
            }
            break;

        case BMA400_SPI_INTF:
            // Begin transmission
            SPI.beginTransaction(SPISettings(interfaceData->spiClockFrequency, MSBFIRST, SPI_MODE0));
            digitalWrite(interfaceData->spiCSPin, LOW);
            
            // Write the address
            SPI.transfer(regAddress);
            
            // Write all the data
            for(uint32_t i = 0; i < numBytes; i++)
            {
                SPI.transfer(dataBuffer[i]);
            }

            // End transmission
            digitalWrite(interfaceData->spiCSPin, HIGH);
            SPI.endTransaction();
            break;
    }

    return BMA400_OK;
}

void BMA400::usDelay(uint32_t period, void* interfacePtr)
{
    delayMicroseconds(period);
}