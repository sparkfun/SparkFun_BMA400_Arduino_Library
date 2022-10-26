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
    Serial.println("BMA400 Example 4 - Filtering");

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

    // Set OSR (oversampling) to maximum setting. This improves accuracy and
    // reduces noise at the cost of power
    accelerometer.setOSR(BMA400_ACCEL_OSR_SETTING_3);
    
    // The sensor's resolution is based on the measurement range, which defaults
    // to 4g. We can reduce the range to the minimum of 2g to get the best
    // measurement resolution
    accelerometer.setRange(BMA400_RANGE_2G);
    
    // The BMA400 has 2 built-in filters that the raw data passes through.
    // Filter 2 is the only source for the interrupt engine, and its parameters
    // are fixed at 100Hz ODR and 48Hz bandwidth, so it's best to use filter 1.
    // It's also possible to use the dedicated low-pass filter by passing
    // BMA400_DATA_SRC_ACCEL_FILT_LP, however its parameters are fixed at 100Hz
    // ODR and 1Hz bandwidth
    accelerometer.setDataSource(BMA400_DATA_SRC_ACCEL_FILT_1);
    
    // Here we can set the filter 1 bandwidth frequency to 2 possible settings:
    // BMA400_ACCEL_FILT1_BW_0 = 0.48 * ODR
    // BMA400_ACCEL_FILT1_BW_1 = 0.24 * ODR
    // Lower bandwidths result in smoother data filtering
    accelerometer.setFilter1Bandwidth(BMA400_ACCEL_FILT1_BW_1);
    
    // Since the filter bandwidth depends on the ODR (output data rate), we can
    // achieve smaller bandwidths by reducing the ODR from the default of 200Hz.
    // If we instead choose 50Hz with the bandwidth set to 0.24, we achieve a
    // bandwidth of 12Hz. Lowering the ODR also reduces the inherent sensor noise
    accelerometer.setODR(BMA400_ODR_50HZ);
    
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