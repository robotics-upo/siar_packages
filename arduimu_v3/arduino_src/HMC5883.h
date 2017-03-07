/*
    AP_Compass_HMC5883L.cpp - Code based on Arduino Library for HMC5883L I2C magnetometer
 Code by Jordi Mu√±oz and Jose Julio. DIYDrones.com
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 Sensor is conected to I2C port
 Sensor is initialized in Continuos mode (75Hz)
 
 */

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define ModeRegister         0x02
#define DataOutputXMSB       0x03
#define DataOutputXLSB       0x04
#define DataOutputZMSB       0x05
#define DataOutputZLSB       0x06
#define DataOutputYMSB       0x07
#define DataOutputYLSB       0x08
#define StatusRegister       0x09
#define IDRegisterA          0x0A
#define IDRegisterB          0x0B
#define IDRegisterC          0x0C

// default gain value
#define magGain              0x20

// ModeRegister valid modes
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

// ConfigRegA valid measurement configuration bits
#define NormalOperation      0x10
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12

//Sensor variables
int magX;
int magY;
int magZ;


