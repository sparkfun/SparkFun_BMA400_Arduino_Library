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
    Serial.println("BMA400 Example 3 - Interrupts");

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

    // The default ODR (output data rate) is 200Hz, which is too fast to read
    // reasonably. Here we reduce the ODR to the minimum of 12.5Hz
    accelerometer.setODR(BMA400_ODR_12_5HZ);

    // The BMA400 has 2 interrupt pins. All interrupt conditions can be mapped
    // to either pin, so we'll just choose the first one for this example
    accelerometer.setDRDYInterruptChannel(BMA400_INT_CHANNEL_1);

    // Here we configure the INT1 pin to push/pull mode, active high
    accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1);
    
    // Enable DRDY interrupt condition
    accelerometer.enableInterrupt(BMA400_DRDY_INT_EN, true);
    
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

        // Check if this is the "data ready" interrupt condition
        if(interruptStatus & BMA400_ASSERTED_DRDY_INT)
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