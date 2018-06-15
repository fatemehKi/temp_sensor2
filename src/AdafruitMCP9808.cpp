/**************************************************************************/
/*! 
    @file     Adafruit_MCP9808.cpp
    @author   K.Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	I2C Driver for Microchip's MCP9808 I2C Temp sensor
	This is a library for the Adafruit MCP9808 breakout
	----> http://www.adafruit.com/products/1782
		
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!
	@section  HISTORY
    v1.0 - First release
*/
/**************************************************************************/

/*
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

/*
#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif
*/
/*

#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
*/
	
#include "AdafruitMCP9808.h"


/**************************************************************************/
/*! 
    @brief  Instantiates a new MCP9808 class
*/
/**************************************************************************/
Adafruit_MCP9808::Adafruit_MCP9808() 
{
	kI2CBus = 2 ;           // Default I2C bus for Lidar-Lite on Jetson TK1
	error = 0 ;
}


Adafruit_MCP9808::~Adafruit_MCP9808() 
{
	closeAdafruit_MCP9808();
}

/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_MCP9808::openAdafruit_MCP9808() {
      char fileNameBuffer[32];
      
    printf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
   // kI2CFileDescriptor = open("/dev/i2c-%d", O_RDWR);
    if (kI2CFileDescriptor < 0) {
    	printf ("couldn't open");
        // Could not open the file
        error = errno ;
        printf ("errono = %d", error);
        return false ;
    }
    if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kAdafruit_MCP9808I2CAddress) < 0) {
        // Could not open the device on the bus
    	printf("couldn't open2");
        error = errno ;
        return false ;
    }
    return true ;
}
 
void Adafruit_MCP9808::closeAdafruit_MCP9808()
{
    if (kI2CFileDescriptor > 0) {
        close(kI2CFileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        kI2CFileDescriptor = -1 ;
    }
}
/**************************************************************************/
/*! 
    @brief  Reads the 16-bit temperature register and returns the Centigrade
            temperature as a float.
*/
/**************************************************************************/
int Adafruit_MCP9808::readAdafruit_MCP9808(int readRegister)
{
	int toReturn=0;
    toReturn = i2c_smbus_write_byte(kI2CFileDescriptor, readRegister);
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    toReturn = i2c_smbus_read_byte(kI2CFileDescriptor) ;
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}



//*************************************************************************
// Set Sensor to Shutdown-State or wake up (Conf_Register BIT8)
// 1= shutdown / 0= wake up
//*************************************************************************

int Adafruit_MCP9808::writeAdafruit_MCP9808(int writeRegister, int writeValue)
{
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, writeRegister, writeValue);
    // Wait a little bit to make sure it settles
    usleep(10000);
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;

}


int Adafruit_MCP9808::getTemperature()
{
    int ioResult ;
    int msb, lsb ;
    ioResult = writeAdafruit_MCP9808(kAdafruit_MCP9808CommandControlRegister,kAdafruit_MCP9808Measure);
    if (ioResult < 0) {
        return ioResult ;
    }
    ioResult = readAdafruit_MCP9808(kAdafruit_MCP9808CalculateTemperatureMSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        msb = ioResult ;
    }
    ioResult = readAdafruit_MCP9808(kAdafruit_MCP9808CalculateTemperatureLSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        lsb = ioResult ;
    }

    int temperature = (msb << 8) + lsb;

    return temperature;
}


int Adafruit_MCP9808::getHardwareVersion()
{
    return readAdafruit_MCP9808(kAdafruit_MCP9808HardwareVersion) ;
}

// Return the Lidar-Lite software version
int Adafruit_MCP9808::getSoftwareVersion() {
    return readAdafruit_MCP9808(kAdafruit_MCP9808SoftwareVersion) ;
}


// Return the last i/o error
int Adafruit_MCP9808::getError()
{
    return error ;
}


