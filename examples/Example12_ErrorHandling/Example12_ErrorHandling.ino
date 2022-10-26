#include <Wire.h>
#include "SparkFun_BMA400_Arduino_Library.h"

// Create a new sensor object
BMA400 accelerometer;

// I2C address selection. This has been intentionally flipped to show how the
// BMA400_E_COM_FAIL error can appear
//uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT; // 0x14 INTENTIONAL ERROR
uint8_t i2cAddress = BMA400_I2C_ADDRESS_SECONDARY; // 0x15
    
// Every method of the BMA400 class returns an error code indicating whether
// it completed successfully. These error codes are 8-bit signed integers,
// and each value has a specific meaning. 0 indicates success, negative values
// mean failure, and positive values are warnings. If you look in bma400_defs.h,
// you'll find the following:
// 
// #define BMA400_OK                                 INT8_C(0)
// #define BMA400_E_NULL_PTR                         INT8_C(-1)
// #define BMA400_E_COM_FAIL                         INT8_C(-2)
// #define BMA400_E_DEV_NOT_FOUND                    INT8_C(-3)
// #define BMA400_E_INVALID_CONFIG                   INT8_C(-4)
// #define BMA400_W_SELF_TEST_FAIL                   INT8_C(1)
// 
// These can be used to handle different errors in different ways. For example,
// if a BMA400_E_COM_FAIL is returned, the sensor did not communicate properly,
// so you might be able to fix it by trying again
// 
// Because every method returns an error code, we'll create this global variable
// to track them.
int8_t err = BMA400_OK;

// Here we set up a couple parameters for attempting to reconnect to the sensor
#define CONNECT_MAX_RETRIES 10
#define CONNECT_RETRY_INTERVAL 1000
uint8_t connectAttempts = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMA400 Example 12 - Error Handling");

    Serial.println();
    Serial.println("!!! ATTENTION !!!");
    Serial.println();
    Serial.println("This example has some intentional problems to illustrate how those errors can be");
    Serial.println("handled. You are encouraged to read the error messages in order to fix the code!");
    Serial.println();

    // Give hints to the user if requested
    giveHints();

    // Initialize the I2C library
    Wire.begin();

    // Try to connect and initialize the sensor. This has been intentionally
    // commented out to show how the BMA400_E_NULL_PTR error can appear
    // connectAndInit(); // INTENTIONAL ERROR
}

void loop()
{
    // Get measurements from the sensor. This must be called before accessing
    // the acceleration data, otherwise it will never update
    err = accelerometer.getSensorData();
    handleError(err);

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

    // Print once per second
    delay(1000);
}

// This is a helper function that attempts to connect to the BMA400, then
// initialize it with a few configuration options.
void connectAndInit()
{
    // Try to connect with the BMA400
    connectToBMA400();

    // If we get here, then we've connected to the BMA400. It's a good idea to
    // run the self test feature of the BMA400, where it actually pushes the
    // sensing element around to ensure it's working properly. This one function
    // call will automatically test each axis and check whether the measured
    // signals are within acceptable ranges. Note that this resets the sensor
    // after completion, so this should be run before changing any config values
    Serial.println("Running self test...");
    err = accelerometer.selfTest();
    handleError(err);

    // Here's an example of error checking with a configuration change. This is
    // intentionally incorrect to show how the BMA400_E_INVALID_CONFIG error can
    // appear. In order to correctly set the ODR to 200, the number should be
    // replaced with BMA400_ODR_200HZ
    Serial.println("Setting ODR...");
    err = accelerometer.setODR(200); // INTENTIONAL ERROR
    handleError(err);
}

// This is a helper function that will attempt to establish connect with the
// BMA400. If a connection cannot be established, this will retry the connection
// after a short delay. This will retry some number of times until giving up
void connectToBMA400()
{
    // We're starting a new connection attempt, so increment the counter
    connectAttempts++;

    Serial.print("Trying to connect to the BMA400, attempt number ");
    Serial.print(connectAttempts);
    Serial.print("/");
    Serial.print(CONNECT_MAX_RETRIES);
    Serial.println();

    // Try to connect to the sensor and initialize it
    err = accelerometer.beginI2C(i2cAddress);

    // Check the error to see whether it worked
    if(err == BMA400_OK)
    {
        // We've successfully connected to the sensor!
        Serial.println("BMA400 connected!");

        // If the sensor has connection issues in the future, this function will
        // be called again, so we need to reset the connection attempts
        connectAttempts = 0;
        
        // Now we can simply return to where we came from
        return;
    }
    
    // There was a problem connecting, check whether we've maxed out our number
    // of attempts
    if(connectAttempts >= CONNECT_MAX_RETRIES)
    {
        // We've used all our connection attempts, so the problem is unlikely to
        // resovle itself. We'll just give up here
        Serial.println("Max connection attempts made.");
        freeze();
    }

    // We have more connection attempts remaining, see if the error handler can
    // fix the problem
    handleError(err);
}

// This is a helper function that will look at the latest error code and
// determine the best course of action.
void handleError(int8_t errorCode)
{
    switch(errorCode)
    {
        case BMA400_OK:
            // No problem, we can just continue on
            break;
        case BMA400_E_NULL_PTR:
            // Null pointer error. With this library, that usually indicates the
            // sensor hasn't been initialized, so let's try reconnecting to it
            Serial.println("Null pointer! Reconnecting...");
            connectAndInit();
            break;
        case BMA400_E_COM_FAIL:
            // Communication failure, this usually indicates the sensor is not
            // wired up correctly, or the I2C address is wrong. It's possible
            // for this to fix itself (eg. loose wires), so let's just print a
            // message and try again after a short delay
            Serial.println("Communication failure, check wiring and I2C address!");
            delay(CONNECT_RETRY_INTERVAL);
            connectAndInit();
            break;
        case BMA400_E_DEV_NOT_FOUND:
            // Device not found means we got communication with some device, but
            // it wasn't the BMA400. This is not likely to fix itself, so we'll
            // just freeze the program
            Serial.println("Device not found!");
            freeze();
            break;
        case BMA400_E_INVALID_CONFIG:
            // Invalid configuration usually indicates one of the parameters
            // given to a method were not valid. This indicates an issue wih the
            // code, which will definitely not fix itself, so we'll just print a
            // message and freeze up
            Serial.println("Invalid config!");
            freeze();
            break;
        case BMA400_W_SELF_TEST_FAIL:
            // Self test failed. This is a warning that means the sensor is
            // connected and working, however the acceleration measurements are
            // invalid for some reason, and should not be trusted. Everything
            // else is fine, so we'll just continue on
            Serial.println("Self test failure, do not trust measurements!");
            break;
        default:
            // An unknown error code was returned. This indicates an issue with
            // the code itself, so let's just print it and freeze
            Serial.print("Unknown error code! ");
            Serial.println(errorCode);
            freeze();
            break;
    }
}

// This is a helper function that freezes the entire program by entering an
// infinite loop. This should only be used in the event of a total failure that
// cannot be recovered from. It's also a good idea to print something before
// calling this to indicate what the problem was
void freeze()
{
    Serial.println("Freezing program. Read the messages above to find and fix the error!");
    Serial.flush(); // Wait for serial buffer to empty
    while(1);
}

// This is a helper function that will give hints to the user about the
// intentional errors in the code
void giveHints()
{
    char userInput = 0;

    Serial.println("Enter \"1\" to get a hint, or any other key to skip hints");
    Serial.println();

    while(userInput != 'q') // q for quit
    {
        // Wait for user input
        while(Serial.available() == false);
        delay(100); // Extra delay in case characters don't all come at once

        // Read first character of user's input
        userInput = Serial.read();

        // Throw away remaining characters
        while(Serial.available()) Serial.read();

        switch(userInput)
        {
            case '1':
                Serial.println("Hint 1: the I2C address is incorrect in the code!");
                Serial.println("Enter \"2\" to get the next hint, or any other key to skip hints");
                Serial.println();
                break;
            case '2':
                Serial.println("Hint 2: in \"setup()\", \"connectAndInit()\" is commented out!");
                Serial.println("Enter \"3\" to get the next hint, or any other key to skip hints");
                Serial.println();
                break;
            case '3':
                Serial.println("Hint 3: setODR uses \"200\" instead of \"BMA400_ODR_200HZ\"!");
                Serial.println("Enter \"4\" to get the next hint, or any other key to skip hints");
                Serial.println();
                break;
            case '4':
                Serial.println("Hint 4: search for \"INTENTIONAL ERROR\" in the comments of this example");
                Serial.println("That's all the hints, press any key to continue");
                Serial.println();
                break;
            default:
                // Unknown character, skip hints
                userInput = 'q'; // q for quit
                break;
        }
    }
}