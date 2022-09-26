#include <SPI.h>
#include "SparkFun_BMA400_Arduino_Library.h"

// Create a new sensor object
BMA400 accelerometer;

// SPI parameters
uint8_t chipSelectPin = 10;
uint32_t clockFrequency = 100000;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMA400 Example2 begin!");

    // Initialize the SPI library
    SPI.begin();

    // Check if sensor is connected and initialize
    // Clock frequency is optional (defaults to 100kHz)
    while(accelerometer.beginSPI(chipSelectPin, clockFrequency) != BMA400_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMA400 not connected, check wiring and CS pin!");

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

    // Print 10x per second
    delay(100);
}