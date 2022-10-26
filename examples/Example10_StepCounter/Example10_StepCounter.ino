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
    Serial.println("BMA400 Example 10 - Step Counter");

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

    // Here we configure the step counter feature of the BMA400. Step detection
    // and counting is handled entirely by the sensor. The parameters used for
    // step detection can be modified, but it's not recommended and is not well
    // documented; see datasheet for more info. The BMA400 stores the step count
    // as a 24-bit integer, which can be read at any time.
    bma400_step_int_conf config =
    {
        .int_chan = BMA400_INT_CHANNEL_1 // Which pin to use for interrupts
    };
    accelerometer.setStepCounterInterrupt(&config);
    
    // Here we configure the INT1 pin to push/pull mode, active high
    accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1);
    
    // Enable step counter interrupt condition. This must be set to enable step
    // counting at all, even if you don't want interrupts to be generated.
    // In that case,  set the interrupt channel above to BMA400_UNMAP_INT_PIN
    accelerometer.enableInterrupt(BMA400_STEP_COUNTER_INT_EN, true);
    
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
        
        // Check if this is the step interrupt condition
        if(interruptStatus & BMA400_ASSERTED_STEP_INT)
        {
            // Get total step count and detected activity type (running/walking/still)
            uint32_t stepCount = 0;
            uint8_t activityType = 0;
            accelerometer.getStepCount(&stepCount, &activityType);
            
            // Print total step count so far
            Serial.print("Step detected! Step count: ");
            Serial.print(stepCount);
            Serial.print("\t");

            // Print the detected activity type (running/walking/still)
            Serial.print("Detected activity type: ");
            switch(activityType)
            {
                case BMA400_RUN_ACT:
                    Serial.println("Running");
                    break;
                case BMA400_WALK_ACT:
                    Serial.println("Walking");
                    break;
                case BMA400_STILL_ACT:
                    Serial.println("Standing still");
                    break;
                default:
                    Serial.println("Unknown");
                    break;
            }
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