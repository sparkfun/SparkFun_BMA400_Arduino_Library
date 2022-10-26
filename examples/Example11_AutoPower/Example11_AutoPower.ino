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
    Serial.println("BMA400 Example 11 - Auto Power");

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

    // We'll start the sensor off in low power mode while we set the automatic
    // power transition settings. Note that low power is different from sleep,
    // the sensor is still taking measurements in this mode!
    accelerometer.setMode(BMA400_MODE_LOW_POWER);
    
    // Here we configure the sensor to automatically transition from low power
    // to normal mode when significant motion is detected. This will also
    // trigger an interrupt, allowing us to start logging acceleration data
    bma400_wakeup_conf wakeupConfig =
    {
        .wakeup_ref_update = BMA400_UPDATE_ONE_TIME, // Whether to automatically update reference values
        .sample_count = BMA400_SAMPLE_COUNT_1, // Number of samples that exceed threshold to wakeup, up to 8
        .wakeup_axes_en = BMA400_AXIS_XYZ_EN, // Which axes to evaluate for interrupts (X/Y/Z in any combination)
        .int_wkup_threshold = 4, // Upper 8 bits of acceleration (at 4g range (default), 4 = 0.0625g)
        .int_wkup_ref_x = 0, // Upper 8 bits of acceleration
        .int_wkup_ref_y = 0, // Upper 8 bits of acceleration
        .int_wkup_ref_z = 64, // Upper 8 bits of acceleration (at 4g range (default), 64 = 1g)
        .int_chan = BMA400_INT_CHANNEL_1 // Which pin to use for interrupts
    };
    accelerometer.setWakeupInterrupt(&wakeupConfig);
    
    // Here we configure the sensor to automatically enter low power mode after
    // a timer reaches the timeout threshold. With the "time reset" trigger, the
    // timer is reset if the generic 2 interrupt is triggered, allowing us to
    // stay in normal power mode while motion continues
    bma400_auto_lp_conf autoLPConfig =
    {
        .auto_low_power_trigger = BMA400_AUTO_LP_TIME_RESET_EN, // drdy, gen1, timeout, timeout reset
        .auto_lp_timeout_threshold = 400 // 2.5ms resolution (eg. 400 results in 1s)
    };
    accelerometer.setAutoLowPower(&autoLPConfig);
    
    // Here we configure the generic 2 interrupt to reset the auto low power
    // timer while motion is detected
    bma400_gen_int_conf config =
    {
        .gen_int_thres = 5, // 8mg resolution (eg. gen_int_thres=10 results in 40mg)
        .gen_int_dur = 1, // 10ms resolution (eg. gen_int_dur=1 results in 10ms)
        .axes_sel = BMA400_AXIS_XYZ_EN, // Which axes to evaluate for interrupts (X/Y/Z in any combination)
        .data_src = BMA400_DATA_SRC_ACCEL_FILT_2, // Which filter to use (must be 100Hz, datasheet recommends filter 2)
        .criterion_sel = BMA400_ACTIVITY_INT, // Trigger interrupts when active or inactive
        .evaluate_axes = BMA400_ANY_AXES_INT, // How to combine axes for interrupt condition (OR/AND)
        .ref_update = BMA400_UPDATE_EVERY_TIME, // Whether to automatically update reference values
        .hysteresis = BMA400_HYST_48_MG, // Hysteresis acceleration for noise rejection
        .int_thres_ref_x = 0, // Raw 12-bit acceleration value
        .int_thres_ref_y = 0, // Raw 12-bit acceleration value
        .int_thres_ref_z = 512, // Raw 12-bit acceleration value (at 4g range (default), 512 = 1g)
        .int_chan = BMA400_UNMAP_INT_PIN // Which pin to use for interrupts
    };
    accelerometer.setGeneric2Interrupt(&config);
    
    // Here we configure the INT1 pin to push/pull mode, active high
    accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1);
    
    // Enable auto wakeup interrupt condition
    accelerometer.enableInterrupt(BMA400_AUTO_WAKEUP_EN, true);
    
    // Enable generic 2 interrupt condition
    accelerometer.enableInterrupt(BMA400_GEN2_INT_EN, true);
    
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

        Serial.print("Interrupt occurred!");
        Serial.print("\t");

        // Get the interrupt status to know which condition triggered
        uint16_t interruptStatus = 0;
        accelerometer.getInterruptStatus(&interruptStatus);
        
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
    // Variable to store the current power mode
    uint8_t powerMode;

    // Loop until the sensor returns to low power mode
    do
    {
        // Get measurements from the sensor. This must be called before
        // accessing the acceleration data, otherwise it will never update
        accelerometer.getSensorData();

        // Print acceleration data
        Serial.print("Acceleration in g's");
        Serial.print("\t");
        Serial.print("X: ");
        Serial.print(accelerometer.data.accelX, 3);
        Serial.print("\t");
        Serial.print("Y: ");
        Serial.print(accelerometer.data.accelY, 3);
        Serial.print("\t");
        Serial.print("Z: ");
        Serial.println(accelerometer.data.accelZ, 3);

        // Print 50x per second
        delay(20);

        // Update sensor's power mode
        accelerometer.getMode(&powerMode);
    } while(powerMode == BMA400_MODE_NORMAL);

    Serial.println("Returning to low power mode");
}

void bma400InterruptHandler()
{
    interruptOccurred = true;
}