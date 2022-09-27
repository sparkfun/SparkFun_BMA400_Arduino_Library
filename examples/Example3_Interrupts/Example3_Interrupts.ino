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
    Serial.println("BMA400 Example3 begin!");

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

    // The default ODR (output data rate) is 200Hz, which is too fast to read
    // reasonably. Here we reduce the ODR to the minimum of 12.5Hz
    err = accelerometer.setODR(BMA400_ODR_12_5HZ);
    if(err != BMA400_OK)
    {
        // ODR setting failed, most likely a communication error (code -2)
        Serial.print("ODR setting failed! Error code: ");
        Serial.println(err);
    }

    // The BMA400 has 2 interrupt pins. All interrupt conditions can be mapped
    // to either pin, so we'll just choose the first one for this example
    err = accelerometer.setDRDYInterruptChannel(BMA400_INT_CHANNEL_1);
    if(err != BMA400_OK)
    {
        // Interrupt channel failed, most likely a communication error (code -2)
        Serial.print("Interrupt channel failed! Error code: ");
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

    // Enable DRDY interrupt condition
    err = accelerometer.enableInterrupt(BMA400_DRDY_INT_EN, true);
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

        // Check if this is the "data ready" interrupt condition
        if(interruptStatus & BMA400_ASSERTED_DRDY_INT)
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