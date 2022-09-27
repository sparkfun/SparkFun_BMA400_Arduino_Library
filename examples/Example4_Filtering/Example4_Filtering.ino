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
    Serial.println("BMA400 Example4 begin!");

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

    // Set OSR (oversampling) to maximum setting. This improves accuracy and
    // reduces noise at the cost of power
    err = accelerometer.setOSR(BMA400_ACCEL_OSR_SETTING_3);
    if(err != BMA400_OK)
    {
        // OSR setting failed, most likely a communication error (code -2)
        Serial.print("OSR setting failed! Error code: ");
        Serial.println(err);
    }

    // The sensor's resolution is based on the measurement range, which defaults
    // to 4g. We can reduce the range to the minimum of 2g to get the best
    // measurement resolution
    err = accelerometer.setRange(BMA400_RANGE_2G);
    if(err != BMA400_OK)
    {
        // Range setting failed, most likely a communication error (code -2)
        Serial.print("Range setting failed! Error code: ");
        Serial.println(err);
    }

    // The BMA400 has 2 built-in filters that the raw data passes through.
    // Filter 2 is the only source for the interrupt engine, and its parameters
    // are fixed at 100Hz ODR and 48Hz bandwidth, so it's best to use filter 1
    // for measurements. It's also possible to use the dedicated low-pass filter
    // by passing BMA400_DATA_SRC_ACCEL_FILT_LP, however its parameters are
    // fixed at 100Hz ODR and 1Hz bandwidth
    err = accelerometer.setDataSource(BMA400_DATA_SRC_ACCEL_FILT_1);
    if(err != BMA400_OK)
    {
        // Data source failed, most likely a communication error (code -2)
        Serial.print("Data source failed! Error code: ");
        Serial.println(err);
    }

    // Here we can set the filter 1 bandwidth frequency to 2 possible settings:
    // BMA400_ACCEL_FILT1_BW_0 = 0.48 * ODR
    // BMA400_ACCEL_FILT1_BW_1 = 0.24 * ODR
    // Lower bandwidths result in smoother data filtering
    err = accelerometer.setFilter1Bandwidth(BMA400_ACCEL_FILT1_BW_1);
    if(err != BMA400_OK)
    {
        // Filter bandwidth failed, most likely a communication error (code -2)
        Serial.print("Filter bandwidth failed! Error code: ");
        Serial.println(err);
    }

    // Since the filter bandwidth depends on the ODR (output data rate), we can
    // achieve smaller bandwidths by reducing the ODR from the default of 200Hz.
    // If we choose the minimum ODR of 12.5Hz with the bandwidth set to 0.24, we
    // achieve a bandwidth of 3Hz. Lowering the ODR also reduces the sensor
    // noise even without any filtering
    err = accelerometer.setODR(BMA400_ODR_12_5HZ);
    if(err != BMA400_OK)
    {
        // ODR setting failed, most likely a communication error (code -2)
        Serial.print("ODR setting failed! Error code: ");
        Serial.println(err);
    }
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