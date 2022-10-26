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
    Serial.println("BMA400 Example 8 - Activity Change");

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

    // Here we configure the activity change detection feature of the BMA400.
    // This evaluates the acceleration data over a certain period defined by
    // act_ch_ntps, and triggers an interrupt if one period is significantly
    // different from the previous period. The threshold for this difference is
    // set by act_ch_thres. See datasheet for more details.
    bma400_act_ch_conf config =
    {
        .act_ch_thres = 10, // 8mg resolution (eg. gen_int_thres=10 results in 80mg)
        .axes_sel = BMA400_AXIS_XYZ_EN, // Which axes to evaluate for interrupts (X/Y/Z in any combination)
        .data_source = BMA400_DATA_SRC_ACC_FILT1, // Which data source to use
        .act_ch_ntps = BMA400_ACT_CH_SAMPLE_CNT_64, // Measurement period (also depends on ODR)
        .int_chan = BMA400_INT_CHANNEL_1 // Which pin to use for interrupts
    };
    accelerometer.setActivityChangeInterrupt(&config);
    
    // Here we configure the INT1 pin to push/pull mode, active high
    accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1);
    
    // Enable activity change interrupt condition
    accelerometer.enableInterrupt(BMA400_ACTIVITY_CHANGE_INT_EN, true);
    
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

        // Check if this is the activity change interrupt condition
        if(interruptStatus & BMA400_ASSERTED_ACT_CH_X)
        {
            Serial.println("Activity changed! (x-axis)");
        }
        else if(interruptStatus & BMA400_ASSERTED_ACT_CH_Y)
        {
            Serial.println("Activity changed! (y-axis)");
        }
        else if(interruptStatus & BMA400_ASSERTED_ACT_CH_Z)
        {
            Serial.println("Activity changed! (z-axis)");
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