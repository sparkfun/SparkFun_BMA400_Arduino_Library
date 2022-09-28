#include <Wire.h>
#include "SparkFun_BMA400_Arduino_Library.h"

// Create a new sensor object
BMA400 accelerometer;

// I2C address selection
uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT; // 0x14
//uint8_t i2cAddress = BMA400_I2C_ADDRESS_SECONDARY; // 0x15

// Pin used for interrupt detection
int interruptPin = 2;

// Flag to know when interrupts occur
volatile bool interruptOccurred = false;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMA400 Example11 begin!");

    // Initialize the I2C library
    Wire.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x14)
    while(accelerometer.beginI2C(i2cAddress) != BMA400_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMA400 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMA400 connected!");

    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // We'll start the sensor off in low power mode while we set the automatic
    // power transition settings
    err = accelerometer.setMode(BMA400_MODE_LOW_POWER);
    if(err != BMA400_OK)
    {
        // Power mode failed, most likely a communication error (code -2)
        Serial.print("Power mode failed! Error code: ");
        Serial.println(err);
    }

    // Here we configure the sensor to automatically transition from low power
    // to normal mode when significant motion is detected. This will also
    // trigger an interrupt, allowing us to start logging acceleration data
    bma400_wakeup_conf wakeupConfig =
    {
        .wakeup_ref_update = BMA400_UPDATE_EVERY_TIME, // Whether to automatically update reference values
        .sample_count = BMA400_SAMPLE_COUNT_1, // Number of samples that exceed threshold to wakeup, up to 8
        .wakeup_axes_en = BMA400_AXIS_XYZ_EN, // Which axes to evaluate for interrupts (X/Y/Z in any combination)
        .int_wkup_threshold = 16, // Upper 8 bits of acceleration (at 4g range (default), 16 = 0.25g)
        .int_wkup_ref_x = 0, // Upper 8 bits of acceleration
        .int_wkup_ref_y = 0, // Upper 8 bits of acceleration
        .int_wkup_ref_z = 64, // Upper 8 bits of acceleration (at 4g range (default), 64 = 1g)
        .int_chan = BMA400_INT_CHANNEL_1 // Which pin to use for interrupts
    };
    err = accelerometer.setWakeupInterrupt(&wakeupConfig);
    if(err != BMA400_OK)
    {
        // Wakeup settings failed, most likely a communication error (code -2)
        Serial.print("Wakeup settings failed! Error code: ");
        Serial.println(err);
    }

    // Here we configure the sensor to automatically enter low power mode after
    // a timer reaches the timeout threshold. With the "time reset" trigger, the
    // timer is reset if the generic 2 interrupt is triggered, allowing us to
    // stay in normal power mode while motion continues
    bma400_auto_lp_conf autoLPConfig =
    {
        .auto_low_power_trigger = BMA400_AUTO_LP_TIME_RESET_EN, // drdy, gen1, timeout, timeout reset
        .auto_lp_timeout_threshold = 400 // 2.5ms resolution (eg. 400 results in 1s)
    };
    err = accelerometer.setAutoLowPower(&autoLPConfig);
    if(err != BMA400_OK)
    {
        // Low power settings failed, most likely a communication error (code -2)
        Serial.print("Low power settings failed! Error code: ");
        Serial.println(err);
    }

    // Here we configure the generic 2 interrupt to reset the auto low power
    // timer while motion is detected
    bma400_gen_int_conf config =
    {
        .gen_int_thres = 10, // 8mg resolution (eg. gen_int_thres=10 results in 80mg)
        .gen_int_dur = 100, // 10ms resolution (eg. gen_int_dur=100 results in 1s)
        .axes_sel = BMA400_AXIS_XYZ_EN, // Which axes to evaluate for interrupts (X/Y/Z in any combination)
        .data_src = BMA400_DATA_SRC_ACCEL_FILT_2, // Which filter to use (must be 100Hz, datasheet recommends filter 2)
        .criterion_sel = BMA400_ACTIVITY_INT, // Trigger interrupts when active or inactive
        .evaluate_axes = BMA400_ANY_AXES_INT, // How to combine axes for interrupt condition (OR/AND)
        .ref_update = BMA400_UPDATE_EVERY_TIME, // Whether to automatically update reference values
        .hysteresis = BMA400_HYST_96_MG, // Hysteresis acceleration for noise rejection
        .int_thres_ref_x = 0, // Raw 12-bit acceleration value
        .int_thres_ref_y = 0, // Raw 12-bit acceleration value
        .int_thres_ref_z = 512, // Raw 12-bit acceleration value (at 4g range (default), 512 = 1g)
        .int_chan = BMA400_UNMAP_INT_PIN // Which pin to use for interrupts
    };
    err = accelerometer.setGeneric2Interrupt(&config);
    if(err != BMA400_OK)
    {
        // Interrupt settings failed, most likely a communication error (code -2)
        Serial.print("Interrupt settings failed! Error code: ");
        Serial.println(err);
    }

    // Here we configure the INT1 pin to push/pull mode, active high
    err = accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1);
    if(err != BMA400_OK)
    {
        // Interrupt pin mode failed, most likely a communication error (code -2)
        Serial.print("Interrupt pin mode failed! Error code: ");
        Serial.println(err);
    }

    // Enable auto wakeup interrupt condition
    err = accelerometer.enableInterrupt(BMA400_AUTO_WAKEUP_EN, true);
    if(err != BMA400_OK)
    {
        // Interrupt enable failed, most likely a communication error (code -2)
        Serial.print("Interrupt enable failed! Error code: ");
        Serial.println(err);
    }

    // Enable generic 2 interrupt condition
    err = accelerometer.enableInterrupt(BMA400_GEN2_INT_EN, true);
    if(err != BMA400_OK)
    {
        // Interrupt enable failed, most likely a communication error (code -2)
        Serial.print("Interrupt enable failed! Error code: ");
        Serial.println(err);
    }

    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), bma400InterruptHandler, RISING);
}

void loop()
{
    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.print("Interrupt occurred!\t\t");

        // Variable to track errors returned by API calls
        int8_t err = BMA400_OK;

        // Get the interrupt status to know which condition triggered
        uint16_t interruptStatus = 0;
        err = accelerometer.getInterruptStatus(&interruptStatus);
        if(err != BMA400_OK)
        {
            // Status get failed, most likely a communication error (code -2)
            Serial.print("Get interrupt status failed! Error code: ");
            Serial.println(err);
            return;
        }

        // Check if this is the auto wakeup interrupt condition
        if(interruptStatus & BMA400_ASSERTED_WAKEUP_INT)
        {
            Serial.println("Motion detected!");
            logAccelData();
        }
        else
        {
            Serial.println("Wrong interrupt condition!");
        }
    }
}

void logAccelData()
{
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Variable to store the current power mode
    uint8_t powerMode = BMA400_MODE_NORMAL;

    // Loop until the sensor returns to low power mode
    while (powerMode == BMA400_MODE_NORMAL)
    {
        // Get measurements from the sensor
        BMA400_SensorData data = {0};
        int8_t err = accelerometer.getSensorData(&data);

        // Check whether data was acquired successfully
        if(err == BMA400_OK)
        {
            // Acquisistion succeeded, print acceleration data
            Serial.print("Acceleration in g's - ");
            Serial.print("X: ");
            Serial.print(data.x, 2);
            Serial.print("\t\t");
            Serial.print("Y: ");
            Serial.print(data.y, 2);
            Serial.print("\t\t");
            Serial.print("Z: ");
            Serial.print(data.z, 2);
        }
        else
        {
            // Acquisition failed, most likely a communication error (code -2)
            Serial.print("Error getting data from sensor! Error code: ");
            Serial.println(err);

            // If we got an error here, it will most likely persist, so let's
            // give up and go back to waiting
            return;
        }

        // Print 10x per second
        delay(100);

        // Check whether sensor returned to low power mode
        err = accelerometer.getMode(&powerMode);
        if(err == BMA400_OK)
        {
            // Get mode failed, most likely a communication error (code -2)
            Serial.print("Error getting power mode! Error code: ");
            Serial.println(err);

            // If we got an error here, it will most likely persist, so let's
            // give up and go back to waiting
            return;
        }
    }
    
}

void bma400InterruptHandler()
{
    interruptOccurred = true;
}