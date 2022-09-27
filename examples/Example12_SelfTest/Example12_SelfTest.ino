#include <Wire.h>
#include "SparkFun_BMA400_Arduino_Library.h"

// Create a new sensor object
BMA400 accelerometer;

// I2C address selection
uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT; // 0x14
//uint8_t i2cAddress = BMA400_I2C_ADDRESS_SECONDARY; // 0x15

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMA400 Example1 begin!");

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

    // The BMA400 has a built-in self test feature, where it actually pushes the
    // sensing element around to ensure it's working properly. This one function
    // call will automatically test each axis and check whether the measured
    // signals are within acceptable ranges. Note that this does change some of
    // the configuration parameters, so you may need to reset those (see
    // datasheet for details)
    err = accelerometer.selfTest();
    if(err == BMA400_OK)
    {
        Serial.println("Self test passed!");
    }
    else
    {
        Serial.println("Self test failed!");
    }
}

void loop()
{
    // Nothing to do
}