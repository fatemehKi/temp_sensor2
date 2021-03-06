/**************************************************************************/
/*! 
    @file     Adafruit_MCP9808.h
    @author   K. Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	This is a library for the Adafruit MCP9808 Temp Sensor breakout board
	----> http://www.adafruit.com/products/1782
	
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!
	@section  HISTORY
    v1.0  - First release
*/
/**************************************************************************/

#ifndef ADAFRUITMCP9808_H
#define ADAFRUITMCP9808_H

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <i2c/smbus.h>
/*
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
*/
//#include <Wire.h>


#define kAdafruit_MCP9808I2CAddress              0x1b

// Internal Control Registers 
#define kAdafruit_MCP9808CommandControlRegister    0x00    // Command Control Register
//#define kLidarLiteVelocityMeasurementOutput     0x09    // Velocity [Read Only]: in .1 meters/sec (8 bit signed value)
// High byte set means read two bytes
//#define kAdafruit_MCP9808CalculateTemperatureMSB  0x8f    // Calculated distance in cm (difference between signal and reference delay)
                                                        // High byte of calculated delay of signal [Read Only]: reference – calculated after correlation record processing
                                                        // If the returned MSB is 1 then the reading is not considered valid.

#define kAdafruit_MCP9808CalculateTemperatureLSB   0x02    // Low byte of calculated delay of signal [Read Only]: reference – calculated after correlation record processing
#define kAdafruit_MCP9808CalculateTemperatureMSB   0x03    // Previous high byte of calculated delay of signal
//#define kLidarLitePreviousMeasuredDistanceLSB   0x15    // Previous low byte of calculated delay of signal
//#define kMCP9808_REG_AMBIENT_TEMP       0x05

// External Control Registers
#define kAdafruit_MCP9808HardwareVersion               0x41    // Hardware Version: revisions begin with 0x01
#define kAdafruit_MCP9808SoftwareVersion               0x4f    // Software Version: Revisions begin with 0x01

// Register Command
#define kAdafruit_MCP9808Measure                       0x04    // Take acquisition & correlation processing with DC correction

class Adafruit_MCP9808
{
public:
    unsigned char kI2CBus ;         // I2C bus of the Lidar-Lite
    int kI2CFileDescriptor ;        // File Descriptor to the Lidar-Lite
    int error ;
    Adafruit_MCP9808();
    ~Adafruit_MCP9808();
    bool openAdafruit_MCP9808();                   // Open the I2C bus to the Lidar-Lite
    void closeAdafruit_MCP9808();                   // Close the I2C bus to the Lidar-Lite
    int writeAdafruit_MCP9808(int writeRegister,int writeValue) ;
    int readAdafruit_MCP9808(int readRegister) ;
    int getTemperature() ;
    //int getPreviousDistance() ;
    //int getVelocity() ;
    int getHardwareVersion() ;
    int getSoftwareVersion() ;
    int getError() ;
  //  void AdafruitMCP9808::shutdown(void);
   // void AdafruitMCP9808::wake(void);

};

#endif // Adafruit_MCP9808
