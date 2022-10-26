#include "SparkFun_BMA400_Arduino_Library.h"

/// @brief Default constructor
BMA400::BMA400()
{
    // Nothing to do
}

/// @brief Begin communication with the sensor over I2C, initialize it, and set
/// default config parameters
/// @param address I2C address of sensor
/// @param wirePort I2C port to use for communication, defaults to Wire
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::beginI2C(uint8_t address, TwoWire& wirePort)
{
    // Check whether address is valid option
    if(address != BMA400_I2C_ADDRESS_DEFAULT && address != BMA400_I2C_ADDRESS_SECONDARY)
    {
        // Invalid option, don't do anything
        return BMA400_E_INVALID_CONFIG;
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

/// @brief Begin communication with the sensor over SPI, initialize it, and set
/// default config parameters
/// @param csPin Chip select pin of sensor
/// @param clockFrequency SPI clock frequency
/// @return Error code. 0 means success, negative means failure
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

/// @brief Checks whether sensor is connected, sends soft reset command,
/// initializes sensor, then sets default config parameters
/// @return Error code. 0 means success, negative means failure
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
    if(err != BMA400_OK) return err;

    // Initialize the sensor
    err = bma400_init(&sensor);
    if(err != BMA400_OK) return err;

    // Set to normal power mode
    return setMode(BMA400_MODE_NORMAL);
}

/// @brief Sets power mode of sensor
/// @param mode Sensor power mode. Assignable values are:
///     BMA400_MODE_NORMAL (Default)
///     BMA400_MODE_SLEEP
///     BMA400_MODE_LOW_POWER
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setMode(uint8_t mode)
{
    return bma400_set_power_mode(mode, &sensor);
}

/// @brief Gets power mode of sensor
/// @param mode Sensor power mode. Assignable values are:
///     BMA400_MODE_NORMAL (Default)
///     BMA400_MODE_SLEEP
///     BMA400_MODE_LOW_POWER
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getMode(uint8_t* mode)
{
    return bma400_get_power_mode(mode, &sensor);
}

/// @brief Sets auto wakeup config of sensor
/// @param config Auto wakeup config struct, see bma400_auto_wakeup_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setAutoWakeup(bma400_auto_wakeup_conf* config)
{
    bma400_device_conf deviceConfig =
    {
        .type = BMA400_AUTOWAKEUP_TIMEOUT,
        .param = {.auto_wakeup = *config}
    };
    return bma400_set_device_conf(&deviceConfig, 1, &sensor);
}

/// @brief Sets auto low power config of sensor
/// @param config Auto low power config struct, see bma400_auto_lp_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setAutoLowPower(bma400_auto_lp_conf* config)
{
    bma400_device_conf deviceConfig =
    {
        .type = BMA400_AUTO_LOW_POWER,
        .param = {.auto_lp = *config}
    };
    return bma400_set_device_conf(&deviceConfig, 1, &sensor);
}

/// @brief Sets just one parameter of bma400_sensor_conf
/// @param param Parameter to set, see BMA400_AccelParam
/// @param val Value assigned to that parameter
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setAccelParam(BMA400_AccelParam param, uint8_t val)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current accel config so we don't accidentally change anything
    bma400_sensor_conf config;
    config.type = BMA400_ACCEL;
    err = bma400_get_sensor_conf(&config, 1, &sensor);
    if(err != BMA400_OK) return err;

    // Change requested parameter to new value if it's valid
    switch(param)
    {
        case BMA400_ODR:
            if(val < BMA400_ODR_12_5HZ || val > BMA400_ODR_800HZ) return BMA400_E_INVALID_CONFIG;
            config.param.accel.odr = val;
            break;
        case BMA400_RANGE:
            if(val > BMA400_RANGE_16G) return BMA400_E_INVALID_CONFIG;
            config.param.accel.range = val;
            break;
        case BMA400_DATA_SRC:
            if(val > BMA400_DATA_SRC_ACCEL_FILT_LP) return BMA400_E_INVALID_CONFIG;
            config.param.accel.data_src = val;
            break;
        case BMA400_OSR:
            if(val > BMA400_ACCEL_OSR_SETTING_3) return BMA400_E_INVALID_CONFIG;
            config.param.accel.osr = val;
            break;
        case BMA400_OSR_LP:
            if(val > BMA400_ACCEL_OSR_SETTING_3) return BMA400_E_INVALID_CONFIG;
            config.param.accel.osr_lp = val;
            break;
        case BMA400_FILT1_BW:
            if(val > BMA400_ACCEL_FILT1_BW_1) return BMA400_E_INVALID_CONFIG;
            config.param.accel.filt1_bw = val;
            break;
        case BMA400_INT_CHAN:
            if(val > BMA400_MAP_BOTH_INT_PINS) return BMA400_E_INVALID_CONFIG;
            config.param.accel.int_chan = (bma400_int_chan) val;
            break;
        default:
            return BMA400_E_INVALID_CONFIG;
            break;
    }

    // Set new config
    return bma400_set_sensor_conf(&config, 1, &sensor);
}

/// @brief Gets just one parameter of bma400_sensor_conf
/// @param param Parameter to get, see BMA400_AccelParam
/// @param val Value assigned to that parameter
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getAccelParam(BMA400_AccelParam param, uint8_t* val)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current accel config
    bma400_sensor_conf config;
    config.type = BMA400_ACCEL;
    err = bma400_get_sensor_conf(&config, 1, &sensor);
    if(err != BMA400_OK) return err;

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
            return BMA400_E_INVALID_CONFIG;
            break;
    }

    return BMA400_OK;
}

/// @brief Sets measurement range
/// @param range Sensor range, assignable values are:
///     BMA400_RANGE_2G
///     BMA400_RANGE_4G (Default)
///     BMA400_RANGE_8G
///     BMA400_RANGE_16G
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setRange(uint8_t range)
{
    return setAccelParam(BMA400_RANGE, range);
}

/// @brief Gets measurement range
/// @param range Sensor range, assignable values are:
///     BMA400_RANGE_2G
///     BMA400_RANGE_4G (Default)
///     BMA400_RANGE_8G
///     BMA400_RANGE_16G
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getRange(uint8_t* range)
{
    return getAccelParam(BMA400_RANGE, range);
}

/// @brief Sets output data rate
/// @param odr Output data rate, assignable values are:
///     BMA400_ODR_12_5HZ
///     BMA400_ODR_25HZ
///     BMA400_ODR_50HZ
///     BMA400_ODR_100HZ
///     BMA400_ODR_200HZ (Default)
///     BMA400_ODR_400HZ
///     BMA400_ODR_800HZ
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setODR(uint8_t odr)
{
    return setAccelParam(BMA400_ODR, odr);
}

/// @brief Gets output data rate
/// @param odr Output data rate, assignable values are:
///     BMA400_ODR_12_5HZ
///     BMA400_ODR_25HZ
///     BMA400_ODR_50HZ
///     BMA400_ODR_100HZ
///     BMA400_ODR_200HZ (Default)
///     BMA400_ODR_400HZ
///     BMA400_ODR_800HZ
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getODR(uint8_t* odr)
{
    return getAccelParam(BMA400_ODR, odr);
}

/// @brief Sets oversampling rate
/// @param osr Oversampling rate, assignable values are:
///     BMA400_ACCEL_OSR_SETTING_0 (Default, lowest power, lowest accuracy)
///     BMA400_ACCEL_OSR_SETTING_1
///     BMA400_ACCEL_OSR_SETTING_2
///     BMA400_ACCEL_OSR_SETTING_3 (Highest power, highest accuracy)
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setOSR(uint8_t osr)
{
    return setAccelParam(BMA400_OSR, osr);
}

/// @brief Gets oversampling rate
/// @param osr Oversampling rate, assignable values are:
///     BMA400_ACCEL_OSR_SETTING_0 (Default, lowest power, lowest accuracy)
///     BMA400_ACCEL_OSR_SETTING_1
///     BMA400_ACCEL_OSR_SETTING_2
///     BMA400_ACCEL_OSR_SETTING_3 (Highest power, highest accuracy)
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getOSR(uint8_t* osr)
{
    return getAccelParam(BMA400_OSR, osr);
}

/// @brief Sets low power oversampling rate
/// @param osrLP Low power oversampling rate, assignable values are:
///     BMA400_ACCEL_OSR_SETTING_0 (Default, lowest power, lowest accuracy)
///     BMA400_ACCEL_OSR_SETTING_1
///     BMA400_ACCEL_OSR_SETTING_2
///     BMA400_ACCEL_OSR_SETTING_3 (Highest power, highest accuracy)
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setOSRLP(uint8_t osrLP)
{
    return setAccelParam(BMA400_OSR_LP, osrLP);
}

/// @brief Gets low power oversampling rate
/// @param osrLP Low power oversampling rate, assignable values are:
///     BMA400_ACCEL_OSR_SETTING_0 (Default, lowest power, lowest accuracy)
///     BMA400_ACCEL_OSR_SETTING_1
///     BMA400_ACCEL_OSR_SETTING_2
///     BMA400_ACCEL_OSR_SETTING_3 (Highest power, highest accuracy)
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getOSRLP(uint8_t* osrLP)
{
    return getAccelParam(BMA400_OSR_LP, osrLP);
}

/// @brief Sets data source
/// @param source Data source, assignable values are:
///     BMA400_DATA_SRC_ACCEL_FILT_1 (Default, variable ODR)
///     BMA400_DATA_SRC_ACCEL_FILT_2 (100Hz ODR)
///     BMA400_DATA_SRC_ACCEL_FILT_LP (100Hz ODR, 1Hz bandwidth)
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setDataSource(uint8_t source)
{
    return setAccelParam(BMA400_DATA_SRC, source);
}

/// @brief Gets data source
/// @param source Data source, assignable values are:
///     BMA400_DATA_SRC_ACCEL_FILT_1 (Default, variable ODR)
///     BMA400_DATA_SRC_ACCEL_FILT_2 (100Hz ODR)
///     BMA400_DATA_SRC_ACCEL_FILT_LP (100Hz ODR, 1Hz bandwidth)
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getDataSource(uint8_t* source)
{
    return getAccelParam(BMA400_DATA_SRC, source);
}

/// @brief Sets filter 1 bandwidth
/// @param bw Filter 1 bandwidth, assignable values are:
///     BMA400_ACCEL_FILT1_BW_0 (Default, 0.48*ODR)
///     BMA400_ACCEL_FILT1_BW_1 (0.24*ODR)
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setFilter1Bandwidth(uint8_t bw)
{
    return setAccelParam(BMA400_FILT1_BW, bw);
}

/// @brief Gets filter 1 bandwidth
/// @param bw Filter 1 bandwidth, assignable values are:
///     BMA400_ACCEL_FILT1_BW_0 (Default, 0.48*ODR)
///     BMA400_ACCEL_FILT1_BW_1 (0.24*ODR)
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getFilter1Bandwidth(uint8_t* bw)
{
    return getAccelParam(BMA400_FILT1_BW, bw);
}

/// @brief Gets step count from sensor
/// @param count Number of steps counted
/// @param activityType Type of activity detected, assignable values are:
///     BMA400_STILL_ACT
///     BMA400_WALK_ACT
///     BMA400_RUN_ACT
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getStepCount(uint32_t* count, uint8_t* activityType)
{
    return bma400_get_steps_counted(count, activityType, &sensor);
}

/// @brief Runs self test procedure. This is a feature of the BMA400, where it
// physically pushes its sensing element around to determine whether it's
// behaving correctly.
/// @return Error code. 0 means success, negative means failure. Can also return
/// BMA400_W_SELF_TEST_FAIL (positive value) if self test completed, but
/// parameters were outside acceptable values
int8_t BMA400::selfTest()
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Run the self test
    err = bma400_perform_self_test(&sensor);
    if(err != BMA400_OK) return err;

    // If the self test is successful, bma400_perform_self_test performs a soft
    // reset of the sensor, so we should return to normal mode
    return setMode(BMA400_MODE_NORMAL);
}

/// @brief Gets acceleration data from the sensor. This must be called to update
/// the data struct
/// @param sensorTime Whether to include sensor time in the data, default false
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getSensorData(bool sensorTime)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Set whether to include sensor time of this measurement
    uint8_t dataSel = sensorTime ? BMA400_DATA_SENSOR_TIME : BMA400_DATA_ONLY;

    // Get raw data from sensor
    bma400_sensor_data rawData;
    err = bma400_get_accel_data(dataSel, &rawData, &sensor);
    if(err != BMA400_OK) return err;

    // Get current measurement range setting
    uint8_t range = 0;
    err = getRange(&range);
    if(err != BMA400_OK) return err;

    // Convert raw data to g's
    convertRawData(&rawData, &data, range);

    return BMA400_OK;
}

/// @brief Converts raw acceleration data to floating point value in g's
/// @param rawData Raw sensor data
/// @param data Output data
/// @param range Sensor measurement range in g's
/// @param bitWidth Number of bits per axis (8 or 12)
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
    data->accelX = rawData->x * rawToGs;
    data->accelY = rawData->y * rawToGs;
    data->accelZ = rawData->z * rawToGs;

    // Convert raw sensor time to milliseconds
    data->sensorTimeMillis = rawData->sensortime * 1000 / BMA400_TICKS_PER_SECOND;
}

/// @brief Gets temperature of the sensor
/// @param temp Temperature in degrees C
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getTemperature(float* temp)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get raw data from sensor
    int16_t rawTemp = 0;
    err = bma400_get_temperature_data(&rawTemp, &sensor);
    if(err != BMA400_OK) return err;

    // Convert raw data to degrees C
    *temp = rawTemp / 10.0;
    return BMA400_OK;
}

/// @brief Sets interrupt pin as push/pull or open drain, and active high or low
/// @param channel Which pin to configure, see bma400_int_chan
/// @param mode 
///     BMA400_INT_PUSH_PULL_ACTIVE_0
///     BMA400_INT_PUSH_PULL_ACTIVE_1 (Default)
///     BMA400_INT_OPEN_DRIVE_ACTIVE_0
///     BMA400_INT_OPEN_DRIVE_ACTIVE_1
/// @return Error code. 0 means success, negative means failure
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

/// @brief Enables interrupt condition
/// @param intType Interrupt condition, see bma400_int_type
/// @param enable Whether to enable or disable this condition
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::enableInterrupt(bma400_int_type intType, bool enable)
{
    bma400_int_enable config =
    {
        .type = intType,
        .conf = (uint8_t) (enable ? BMA400_ENABLE : BMA400_DISABLE)
    };
    return bma400_enable_interrupt(&config, 1, &sensor);
}

/// @brief Gets interrupt status flags
/// @param status Interrupt status flags, can include any of the following:
///     BMA400_ASSERTED_WAKEUP_INT
///     BMA400_ASSERTED_ORIENT_CH
///     BMA400_ASSERTED_GEN1_INT
///     BMA400_ASSERTED_GEN2_INT
///     BMA400_ASSERTED_INT_OVERRUN
///     BMA400_ASSERTED_FIFO_FULL_INT
///     BMA400_ASSERTED_FIFO_WM_INT
///     BMA400_ASSERTED_DRDY_INT
///     BMA400_ASSERTED_STEP_INT
///     BMA400_ASSERTED_S_TAP_INT
///     BMA400_ASSERTED_D_TAP_INT
///     BMA400_ASSERTED_ACT_CH_X
///     BMA400_ASSERTED_ACT_CH_Y
///     BMA400_ASSERTED_ACT_CH_Z
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getInterruptStatus(uint16_t* status)
{
    return bma400_get_interrupt_status(status, &sensor);
}

/// @brief Sets the interrupt pin for the data ready interrupt condition
/// @param channel Which pin to use, see bma400_int_chan
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setDRDYInterruptChannel(bma400_int_chan channel)
{
    return setAccelParam(BMA400_INT_CHAN, channel);
}

/// @brief Sets the generic 1 interrupt config
/// @param config Generic 1 interrupt config, see bma400_gen_int_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setGeneric1Interrupt(bma400_gen_int_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_GEN1_INT,
        .param = {.gen_int = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

/// @brief Sets the generic 2 interrupt config
/// @param config Generic 2 interrupt config, see bma400_gen_int_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setGeneric2Interrupt(bma400_gen_int_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_GEN2_INT,
        .param = {.gen_int = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

/// @brief Sets the orientation change interrupt config
/// @param config Orientation change interrupt config, see bma400_orient_int_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setOrientationChangeInterrupt(bma400_orient_int_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_ORIENT_CHANGE_INT,
        .param = {.orient = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

/// @brief Sets the tap detection interrupt config
/// @param config Tap detection interrupt config, see bma400_tap_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setTapInterrupt(bma400_tap_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_TAP_INT,
        .param = {.tap = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

/// @brief Sets the step counter interrupt config
/// @param config Step counter interrupt config, see bma400_step_int_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setStepCounterInterrupt(bma400_step_int_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_STEP_COUNTER_INT,
        .param = {.step_cnt = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

/// @brief Sets the activity change interrupt config
/// @param config Activity change interrupt config, see bma400_act_ch_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setActivityChangeInterrupt(bma400_act_ch_conf* config)
{
    bma400_sensor_conf sensorConfig =
    {
        .type = BMA400_ACTIVITY_CHANGE_INT,
        .param = {.act_ch = *config}
    };
    return bma400_set_sensor_conf(&sensorConfig, 1, &sensor);
}

/// @brief Sets the wakeup interrupt config
/// @param config Wakeup change interrupt config, see bma400_wakeup_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setWakeupInterrupt(bma400_wakeup_conf* config)
{
    bma400_device_conf deviceConfig =
    {
        .type = BMA400_AUTOWAKEUP_INT,
        .param = {.wakeup = *config}
    };
    return bma400_set_device_conf(&deviceConfig, 1, &sensor);
}

/// @brief Sets the FIFO config
/// @param config FIFO config, see bma400_fifo_conf
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::setFIFOConfig(bma400_fifo_conf* config)
{
    // Convert watermark from number of measurements to number of bytes
    (*config).fifo_watermark *= bytesPerFIFOData((*config).conf_regs);

    // Get current FIFO config
    struct bma400_device_conf deviceConfig =
    {
        .type = BMA400_FIFO_CONF,
        .param = {.fifo_conf = *config}
    };
    return bma400_set_device_conf(&deviceConfig, 1, &sensor);
}

/// @brief Gets the number of data samples stored in the FIFO buffer. It can
/// store up to 1kB of data, and the storage format is configurable. Each axis
/// can optionally be stored in the buffer, and each axis can be stored as
/// either 8-bit or 12-bit (takes 2 full bytes).
/// @param numData Number of data samples
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getFIFOLength(uint16_t* numData)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // None of the functions available in bma400.h enable getting the current
    // FIFO length, so the regsiters have to be read manually
    uint8_t data[2] = {0};
    err = bma400_get_regs(BMA400_REG_FIFO_LENGTH, data, 2, &sensor);
    if(err != BMA400_OK) return err;

    uint16_t numBytes = ((uint16_t) data[1] << 8) | data[0];

    // Get current FIFO config
    struct bma400_device_conf config;
    config.type = BMA400_FIFO_CONF;
    err = bma400_get_device_conf(&config, 1, &sensor);
    if(err != BMA400_OK) return err;

    // Compute the total number of bytes for this watermark level
    *numData = numBytes / bytesPerFIFOData(config.param.fifo_conf.conf_regs);

    return BMA400_OK;
}

/// @brief Gets acceleration data out of FIFO buffer
/// @param data Array of data structs, see BMA400_SensorData
/// @param numData Number of data samples to read
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::getFIFOData(BMA400_SensorData* data, uint16_t* numData)
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get current FIFO config
    struct bma400_device_conf config;
    config.type = BMA400_FIFO_CONF;
    err = bma400_get_device_conf(&config, 1, &sensor);
    if(err != BMA400_OK) return err;

    // Short variable name to reference FIFO config flags
    uint8_t* flags = &(config.param.fifo_conf.conf_regs);

    // Compute number of bytes that need to be read out from FIFO
    uint16_t numBytes = *numData * bytesPerFIFOData(*flags);

    // Determine whether sensor time is enabled
    if(*flags & BMA400_FIFO_TIME_EN)
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
    bma400_fifo_data fifoData;
    fifoData.data = byteBuffer;
    fifoData.length = numBytes;
    err = bma400_get_fifo_data(&fifoData, &sensor);
    if(err != BMA400_OK) return err;

    // Create array of raw data
    bma400_sensor_data rawData[*numData];

    // Extract sensor data out of FIFO data
    err = bma400_extract_accel(&fifoData, rawData, numData, &sensor);
    if(err != BMA400_OK) return err;

    // Determine the number of bits per data
    uint8_t bitWidth = ((*flags & BMA400_FIFO_8_BIT_EN) != 0) ? 8 : 12;

    // Get current measurement range setting
    uint8_t range = 0;
    err = getRange(&range);
    if(err != BMA400_OK) return err;

    // Convert raw data to g's for each data frame
    for(uint16_t i = 0; i < *numData; i++)
    {
        // Add sensor time to each data frame before converting
        rawData[i].sensortime = fifoData.fifo_sensor_time;
        convertRawData(&rawData[i], &data[i], range, bitWidth);
    }

    return BMA400_OK;
}

/// @brief Clears all data in FIFO buffer
/// @return Error code. 0 means success, negative means failure
int8_t BMA400::flushFIFO()
{
    return bma400_set_fifo_flush(&sensor);
}

/// @brief Computes the number of bytes per data sample in the FIFO buffer
/// @param fifoFlags FIFO config flags, see bma400_fifo_conf.conf_regs
/// @return Error code. 0 means success, negative means failure
uint8_t BMA400::bytesPerFIFOData(uint8_t fifoFlags)
{
    // Determine how many axes are being stored in the FIFO buffer
    uint8_t numAxes = ((fifoFlags & BMA400_FIFO_X_EN) != 0)
                    + ((fifoFlags & BMA400_FIFO_Y_EN) != 0)
                    + ((fifoFlags & BMA400_FIFO_Z_EN) != 0);
    
    // Determine the number of bytes per axis (2 full bytes for 12-bit mode)
    uint8_t bytesPerAxis = (fifoFlags & BMA400_FIFO_8_BIT_EN) ? 1 : 2;

    // Compute the total number of bytes per data frame. Data frames include 1
    // header byte, plus 1 or 2 bytes per axis
    return 1 + (numAxes * bytesPerAxis);
}

/// @brief Helper function to read sensor registers
/// @param regAddress Start address to read
/// @param dataBuffer Buffer to store register values
/// @param numBytes Number of bytes to read
/// @param interfacePtr Pointer to interface data, see BMA400_InterfaceData
/// @return Error code. 0 means success, negative means failure
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
            return readRegistersI2C(regAddress, dataBuffer, numBytes, interfaceData);
            break;
        case BMA400_SPI_INTF:
            return readRegistersSPI(regAddress, dataBuffer, numBytes, interfaceData);
            break;
        default:
            return BMA400_E_INVALID_CONFIG;
            break;
    }
}

/// @brief Helper function to read sensor registers over I2C
/// @param regAddress Start address to read
/// @param dataBuffer Buffer to store register values
/// @param numBytes Number of bytes to read
/// @param interfaceData Pointer to interface data, see BMA400_InterfaceData
/// @return Error code. 0 means success, negative means failure
BMA400_INTF_RET_TYPE BMA400::readRegistersI2C(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, BMA400_InterfaceData* interfaceData)
{
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

    return BMA400_OK;
}

/// @brief Helper function to read sensor registers over SPI
/// @param regAddress Start address to read
/// @param dataBuffer Buffer to store register values
/// @param numBytes Number of bytes to read
/// @param interfaceData Pointer to interface data, see BMA400_InterfaceData
/// @return Error code. 0 means success, negative means failure
BMA400_INTF_RET_TYPE BMA400::readRegistersSPI(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, BMA400_InterfaceData* interfaceData)
{
    // Start transmission
    SPI.beginTransaction(SPISettings(interfaceData->spiClockFrequency, MSBFIRST, SPI_MODE0));
    digitalWrite(interfaceData->spiCSPin, LOW);
    SPI.transfer(regAddress | 0x80);

    // Read all requested bytes
    for(uint32_t i = 0; i < numBytes; i++)
    {
        dataBuffer[i] = SPI.transfer(0);
    }

    // End transmission
    digitalWrite(interfaceData->spiCSPin, HIGH);
    SPI.endTransaction();

    return BMA400_OK;
}

/// @brief Helper function to write sensor registers
/// @param regAddress Start address to write
/// @param dataBuffer Buffer to store register values
/// @param numBytes Number of bytes to write
/// @param interfacePtr Pointer to interface data, see BMA400_InterfaceData
/// @return Error code. 0 means success, negative means failure
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
            return writeRegistersI2C(regAddress, dataBuffer, numBytes, interfaceData);
            break;
        case BMA400_SPI_INTF:
            return writeRegistersSPI(regAddress, dataBuffer, numBytes, interfaceData);
            break;
        default:
            return BMA400_E_INVALID_CONFIG;
            break;
    }
}

/// @brief Helper function to write sensor registers over I2C
/// @param regAddress Start address to write
/// @param dataBuffer Buffer to store register values
/// @param numBytes Number of bytes to write
/// @param interfaceData Pointer to interface data, see BMA400_InterfaceData
/// @return Error code. 0 means success, negative means failure
BMA400_INTF_RET_TYPE BMA400::writeRegistersI2C(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, BMA400_InterfaceData* interfaceData)
{
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

    return BMA400_OK;
}

/// @brief Helper function to write sensor registers over SPI
/// @param regAddress Start address to write
/// @param dataBuffer Buffer to store register values
/// @param numBytes Number of bytes to write
/// @param interfaceData Pointer to interface data, see BMA400_InterfaceData
/// @return Error code. 0 means success, negative means failure
BMA400_INTF_RET_TYPE BMA400::writeRegistersSPI(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, BMA400_InterfaceData* interfaceData)
{
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

    return BMA400_OK;
}

/// @brief Helper function to delay for some amount of time
/// @param period Number of microseconds to delay
/// @param interfacePtr Pointer to interface data, see BMA400_InterfaceData
void BMA400::usDelay(uint32_t period, void* interfacePtr)
{
    delayMicroseconds(period);
}