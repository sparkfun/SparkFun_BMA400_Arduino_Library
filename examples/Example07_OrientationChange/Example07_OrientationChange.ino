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
    Serial.println("BMA400 Example 7 - Orientation Change");

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

    // Here we configure the orientation change detection feature of the BMA400.
    // It monitors the acceleration, and waits for the measurements to stabalize
    // If the stabalized mesaurements exceed some reference acceleration that we
    // specify, it triggers an interrupt. This is intended for applications like
    // phone screen orientation updating. There are several parameters that can
    // be configured, such as selecting which axes to monitor, hysteresis,
    // automatic reference updates, thresholds, etc.
    bma400_orient_int_conf config =
    {
        .axes_sel = BMA400_AXIS_XYZ_EN, // Which axes to evaluate for interrupts (X/Y/Z in any combination)
        .data_src = BMA400_DATA_SRC_ACCEL_FILT_LP, // Which filter to use (must be either filt2 or filt_lp)
        .ref_update = BMA400_UPDATE_LP_EVERY_TIME, // Whether to automatically update reference values
        .orient_thres = 50, // 8mg resolution (eg. orient_thres=50 results in 400mg)
        .stability_thres = 10, // 8mg resolution (eg. stability_thres=20 results in 80mg)
        .orient_int_dur = 50, // 10ms resolution (eg. gen_int_dur=50 results in 500ms)
        .orient_ref_x = 0, // Raw 12-bit acceleration value
        .orient_ref_y = 0, // Raw 12-bit acceleration value
        .orient_ref_z = 512, // Raw 12-bit acceleration value (at 4g range (default), 512 = 1g)
        .int_chan = BMA400_INT_CHANNEL_1 // Which pin to use for interrupts
    };
    accelerometer.setOrientationChangeInterrupt(&config);
    
    // Here we configure the INT1 pin to push/pull mode, active high
    accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1);
    
    // Enable orientation change interrupt condition
    accelerometer.enableInterrupt(BMA400_ORIENT_CHANGE_INT_EN, true);
    
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

        // Check if this is the orientation change interrupt condition
        if(interruptStatus & BMA400_ASSERTED_ORIENT_CH)
        {
            Serial.println("Orientation changed!");
        }
        else
        {
            Serial.println("Wrong interrupt condition!");
        }
    }
}

void bma400InterruptHandler()
{
    interruptOccurred = true;
}