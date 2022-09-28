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
    Serial.println("BMA400 Example6 begin!");

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

    // Here we configure the generic interrupt feature of the BMA400. It
    // triggers an interrupt when the measured acceleration exceeds some
    // reference acceleration that we can specify. There are several other
    // parameters that can be configured, such as acceleration threshold,
    // hysteresis, automatic reference updates, etc. The BMA400 has 2 generic
    // interrupts that can be configured independently
    bma400_gen_int_conf config =
    {
        .gen_int_thres = 10, // 8mg resolution (eg. gen_int_thres=10 results in 80mg)
        .gen_int_dur = 10, // 10ms resolution (eg. gen_int_dur=10 results in 100ms)
        .axes_sel = BMA400_AXIS_XYZ_EN, // Which axes to evaluate for interrupts (X/Y/Z in any combination)
        .data_src = BMA400_DATA_SRC_ACCEL_FILT_2, // Which filter to use (must be 100Hz, datasheet recommends filter 2)
        .criterion_sel = BMA400_ACTIVITY_INT, // Trigger interrupts when active or inactive
        .evaluate_axes = BMA400_ANY_AXES_INT, // Logical combining of axes for interrupt condition (OR/AND)
        .ref_update = BMA400_UPDATE_MANUAL, // Whether to automatically update reference values
        .hysteresis = BMA400_HYST_96_MG, // Hysteresis acceleration for noise rejection
        .int_thres_ref_x = 0, // Raw 12-bit acceleration value
        .int_thres_ref_y = 0, // Raw 12-bit acceleration value
        .int_thres_ref_z = 512, // Raw 12-bit acceleration value (at 4g range (default), 512 = 1g)
        .int_chan = BMA400_INT_CHANNEL_1 // Which pin to use for interrupts
    };
    err = accelerometer.setGeneric1Interrupt(&config);
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

    // Enable generic 1 interrupt condition
    err = accelerometer.enableInterrupt(BMA400_GEN1_INT_EN, true);
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

        Serial.print("Interrupt occurred!");
        Serial.print("\t");

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

        // Check if this is the generic 1 interrupt condition
        if(interruptStatus & BMA400_ASSERTED_GEN1_INT)
        {
            Serial.println("Motion detected!");
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