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
AdafruitMCP9808::AdafruitMCP9808() 
{
	kI2CBus = 1 ;           // Default I2C bus for Lidar-Lite on Jetson TK1
	error = 0 ;
}


AdafruitMCP9808::~AdafruitMCP9808() 
{
	closeAdafruitMCP9808();
}

/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
bool AdafruitMCP9808::openAdafruitMCP9808() {
      char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
    if (kI2CFileDescriptor < 0) {
        // Could not open the file
        error = errno ;
        return false ;
    }
    if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kAdafruitMCP9808I2CAddress) < 0) {
        // Could not open the device on the bus
        error = errno ;
        return false ;
    }
    return true ;
}
 
void AdafruitMCP9808::closeAdafruitMCP9808()
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
int AdafruitMCP9808::readAdafruitMCP9808(int readRegister)
	int toReturn ;
    toReturn = i2c_smbus_write_byte(kI2CFileDescriptor, readRegister) ;
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

int AdafruitMCP9808::writeAdafruitMCP9808(int writeRegister, int writeValue)
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


int AdafruitMCP9808::getTemperature()
{
    int ioResult ;
    int msb, lsb ;
    ioResult = writeAdafruitMCP9808(kAdafruit_MCP9808CommandControlRegister,AdafruitMCP9808Measure);
    if (ioResult < 0) {
        return ioResult ;
    }
    ioResult = readAdafruitMCP9808(kAdafruitMCP9808CalculateTemperatureMSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        msb = ioResult ;
    }
    ioResult = readAdafruitMCP9808(kAdafruitMCP9808CalculateTemperatureLSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        lsb = ioResult ;
    }

    int temperature = (msb << 8) + lsb ;

    return temperature ;
}


int AdafruitMCP9808::getHardwareVersion()
{
    return readLidarLite(kLidarLiteHardwareVersion) ;
}

// Return the Lidar-Lite software version
int AdafruitMCP9808::getSoftwareVersion() {
    return readLidarLite(kLidarLiteSoftwareVersion) ;
}


// Return the last i/o error
int AdafruitMCP9808::getError()
{
    return error ;
}

/*
void AdafruitMCP9808::shutdown(void)
{
  shutdown_wake(1);
}

void AdafruitMCP9808::wake(void)
{
  shutdown_wake(0);
  delay(250);
}
*/


/**************************************************************************/
/*! 
    @brief  Low level 16 bit read and write procedures!
*/
/**************************************************************************/
/*
void Adafruit_MCP9808::write16(uint8err_t reg, uint16_t value) {
    Wire.beginTransmission(_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.write(value >> 8);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

uint16_t Adafruit_MCP9808::read16(uint8_t reg) {
  uint16_t val;

  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  
  Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)2);
  val = Wire.read();
  val <<= 8;
  val |= Wire.read();  
  return val;  
}*/
