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

// Create a buffer for FIFO data
const uint16_t numSamples = 25;
BMA400_SensorData fifoData[numSamples];

// Track FIFO length to give progress updates
uint8_t previousFIFOLength = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMA400 Example5 begin!");

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

    // Here we set the configuration flags for the FIFO buffer. The following
    // flags are available:
    // BMA400_FIFO_AUTO_FLUSH   - Flush FIFO when power mode changes
    // BMA400_FIFO_STOP_ON_FULL - Stop storing data when FIFO is full
    // BMA400_FIFO_TIME_EN      - Log sensor time when FIFO is read out
    // BMA400_FIFO_DATA_SRC     - Store data from filter 2 instead of filter 1
    // BMA400_FIFO_8_BIT_EN     - Store data with only 8 bits instead of 12
    // BMA400_FIFO_X_EN         - Store x-axis data
    // BMA400_FIFO_Y_EN         - Store y-axis data
    // BMA400_FIFO_Z_EN         - Store z-axis data
    uint8_t flags = BMA400_FIFO_X_EN | BMA400_FIFO_Y_EN | BMA400_FIFO_Z_EN;
    err = accelerometer.setFIFOConfigFlags(flags);
    if(err != BMA400_OK)
    {
        // FIFO config flags failed, most likely a communication error (code -2)
        Serial.print("FIFO config flags failed! Error code: ");
        Serial.println(err);
    }
    
    // Set watermark level to trigger an interrupt after numSamples measurements
    err = accelerometer.setFIFOWatermark(numSamples);
    if(err != BMA400_OK)
    {
        // Watermark failed, most likely a communication error (code -2)
        Serial.print("Watermark failed! Error code: ");
        Serial.println(err);
    }

    // The BMA400 has 2 interrupt pins. All interrupt conditions can be mapped
    // to either pin, so we'll just choose the first one for this example
    err = accelerometer.setFIFOWatermarkInterruptChannel(BMA400_INT_CHANNEL_1);
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

    // Enable FIFO watermark interrupt condition
    err = accelerometer.enableInterrupt(BMA400_FIFO_WM_INT_EN, true);
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
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    // Get number of data samples currently stored in FIFO buffer
    uint16_t currentFIFOLength = 0;
    err = accelerometer.getFIFOLength(&currentFIFOLength);
    if(err != BMA400_OK)
    {
        // FIFO length failed, most likely a communication error (code -2)
        Serial.print("FIFO length failed! Error code: ");
        Serial.println(err);

        // If getFIFOLength() failed this time, it will most likely fail next
        // time. So let's wait a bit before trying again
        delay(1000);
    }

    // Check whether number of samples in FIFO buffer has changed
    if(previousFIFOLength != currentFIFOLength)
    {
        // Update FIFO length
        previousFIFOLength = currentFIFOLength;

        // Print current FIFO length
        Serial.print("FIFO Length: ");
        Serial.print(currentFIFOLength);
        Serial.print("/");
        Serial.println(numSamples);

        // If the buffer length goes beyond the watermark level, then an
        // interrupt was missed. This example will likely run into issues,
        // so we'll just clear the FIFO buffer
        if(currentFIFOLength > numSamples)
        {
            Serial.println("Too many samples in FIFO buffer, flushing...");
            
            err = accelerometer.flushFIFO();
            if(err != BMA400_OK)
            {
                // FIFO flush failed, most likely a communication error (code -2)
                Serial.print("FIFO flush failed! Error code: ");
                Serial.println(err);
            }
        }
    }

    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.println("Interrupt occurred!");

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

        // Check if this is the FIFO watermerk interrupt condition
        if(interruptStatus & BMA400_ASSERTED_FIFO_WM_INT)
        {
            // Get FIFO data from the sensor
            uint16_t samplesRead = numSamples;
            err = accelerometer.getFIFOData(fifoData, &samplesRead);
            if(err != BMA400_OK)
            {
                // FIFO data get failed, most likely a communication error (code -2)
                Serial.print("Get FIFO data failed! Error code: ");
                Serial.println(err);
                return;
            }

            // samplesRead will be changed to the number of data frames actually
            // read from the FIFO buffer. Check whether it's equal to numSamples
            if(samplesRead != numSamples)
            {
                // Most likely didn't have enough data frames in FIFO buffer.
                // This can happen if control frames are inserted into the FIFO
                // buffer, which occurs when certain configuration changes occur
                Serial.print("Unexpected number of samples read from FIFO: ");
                Serial.println(samplesRead);
            }

            // Print out all acquired data
            for(uint16_t i = 0; i < samplesRead; i++)
            {
                Serial.print("Acceleration in g's");
                Serial.print("\t");
                Serial.print("X: ");
                Serial.print(fifoData[i].x, 3);
                Serial.print("\t");
                Serial.print("Y: ");
                Serial.print(fifoData[i].y, 3);
                Serial.print("\t");
                Serial.print("Z: ");
                Serial.println(fifoData[i].z, 3);
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