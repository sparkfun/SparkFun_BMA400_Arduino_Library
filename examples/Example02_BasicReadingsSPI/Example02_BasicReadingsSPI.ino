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
    Serial.println("BMA400 Example 2 - Basic Readings SPI");

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
    // Get measurements from the sensor. This must be called before accessing
    // the acceleration data, otherwise it will never update
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
}