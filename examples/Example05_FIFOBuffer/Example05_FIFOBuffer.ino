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
const uint16_t numSamples = 15;
BMA400_SensorData fifoData[numSamples];

// Track FIFO length to give progress updates
uint8_t previousFIFOLength = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMA400 Example 5 - FIFO Buffer");

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

    // Here we set the config parameters for the FIFO buffer. There are several
    // flags that we can set, which are listed below. Additionally, we can set
    // the watermark level, and choose where to route the interrupt conditions.
    // conf_regs flags:
    // BMA400_FIFO_AUTO_FLUSH   - Flush FIFO when power mode changes
    // BMA400_FIFO_STOP_ON_FULL - Stop storing data when FIFO is full
    // BMA400_FIFO_TIME_EN      - Log sensor time when FIFO is read out
    // BMA400_FIFO_DATA_SRC     - Store data from filter 2 instead of filter 1
    // BMA400_FIFO_8_BIT_EN     - Store data with only 8 bits instead of 12
    // BMA400_FIFO_X_EN         - Store x-axis data
    // BMA400_FIFO_Y_EN         - Store y-axis data
    // BMA400_FIFO_Z_EN         - Store z-axis data
    bma400_fifo_conf config = 
    {
        .conf_regs = BMA400_FIFO_X_EN | BMA400_FIFO_Y_EN | BMA400_FIFO_Z_EN,
        .conf_status = BMA400_ENABLE,
        .fifo_watermark = numSamples,
        .fifo_full_channel = BMA400_UNMAP_INT_PIN,
        .fifo_wm_channel = BMA400_INT_CHANNEL_1
    };
    accelerometer.setFIFOConfig(&config);

    // Here we configure the INT1 pin to push/pull mode, active high
    accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1);
    
    // Enable FIFO watermark interrupt condition
    accelerometer.enableInterrupt(BMA400_FIFO_WM_INT_EN, true);
    
    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), bma400InterruptHandler, RISING);
}

void loop()
{
    // Get number of data samples currently stored in FIFO buffer
    uint16_t currentFIFOLength = 0;
    accelerometer.getFIFOLength(&currentFIFOLength);
    
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
            accelerometer.flushFIFO();
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
        accelerometer.getInterruptStatus(&interruptStatus);
        
        // Check if this is the FIFO watermerk interrupt condition
        if(interruptStatus & BMA400_ASSERTED_FIFO_WM_INT)
        {
            // Get FIFO data from the sensor
            uint16_t samplesRead = numSamples;
            accelerometer.getFIFOData(fifoData, &samplesRead);
            
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
                Serial.print(fifoData[i].accelX, 3);
                Serial.print("\t");
                Serial.print("Y: ");
                Serial.print(fifoData[i].accelY, 3);
                Serial.print("\t");
                Serial.print("Z: ");
                Serial.println(fifoData[i].accelZ, 3);
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