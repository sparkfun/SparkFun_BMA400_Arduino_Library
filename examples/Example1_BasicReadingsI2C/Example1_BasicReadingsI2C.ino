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
}

void loop()
{
    // Get measurements from the sensor
    BMA400_SensorData data = {0};
    int8_t err = accelerometer.getSensorData(&data);

    // Check whether data was acquired successfully
    if(err == BMA400_OK)
    {
        // Acquisistion succeeded, print acceleration data
        Serial.print("Acceleration in g's");
        Serial.print("\t");
        Serial.print("X: ");
        Serial.print(data.x, 3);
        Serial.print("\t");
        Serial.print("Y: ");
        Serial.print(data.y, 3);
        Serial.print("\t");
        Serial.print("Z: ");
        Serial.println(data.z, 3);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor! Error code: ");
        Serial.println(err);
    }

    // Print 10x per second
    delay(100);
}