// C Library for interfacing the TPS55289 Buck-Boost Converter with RP2040
/*
MIT License
Copyright (c) 2024 Krishna Swaroop

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef TPS55289_H
#define TPS55289_H


#include "pico/stdlib.h"

// Register Addresses
#define TPS55289_REF_VOLTAGE_LSB_ADDR   0x00
#define TPS55289_REF_VOLTAGE_MSB_ADDR   0x01
#define TPS55289_IOUT_LIMIT_ADDR        0x02
#define TPS55289_VOUT_SR_ADDR           0x03
#define TPS55289_VOUT_FS_ADDR           0x04
#define TPS55289_CDC_ADDR               0x05
#define TPS55289_MODE_ADDR              0x06
#define TPS55289_STATUS_ADDR            0x07

// Constants
#define INTFB_00                        0.2256
#define INTFB_01                        0.1128
#define INTFB_10                        0.0752
#define INTFB_11                        0.0564


#define TPS55289_I2C_ADDR               0x74
#define TPPS55289_SENSE_RESISTOR        10      // in milliOhms

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Structure Definitions

// Structure for REF Register (0x01)
typedef struct {
    union {
        struct {
            uint8_t     reserved    : 5;
            uint16_t    VREF        : 10;
        };
        uint16_t regValue_16;  
    };
    float VOUT;                          // Stores set output voltage in mV
    float CURRENT_INTFB;                 // Stores currently chosen internal feedback ratio

    uint8_t VREF_LSB    : 8;
    uint8_t VREF_MSB    : 8;
} TPS55289_REF_VOLTAGE_REG;

// Structure for IOUT_LIMIT Register (0x02) [reset = 0b11100100]
typedef struct {
    union {
        struct {
            uint8_t Current_Limit_EN        : 1;
            uint8_t Current_Limit_Setting   : 7;
        };
        uint8_t regValue;
    };
    uint32_t currentLimitAmp;
} TPS55289_IOUT_LIMIT_REG;

// Structure for VOUT_SR Register (0x03) [reset = 0b00000001]
typedef struct {
    union {
        struct {
            uint8_t reserved    : 2;
            uint8_t OCP_DELAY   : 2;
            uint8_t reserved1   : 2;
            uint8_t SR          : 2;
        };
        uint8_t regValue;
    };
    float OCResponseTime;
    float slewRate;
} TPS55289_VOUT_SR_REG;

// Structure for VOUT_FS Register (0x04) [reset = 0b00000011]
typedef struct {
    union {
        struct {
            uint8_t FB          : 1;        // 0 if internal; 1 if external
            uint8_t reserved    : 5;
            uint8_t INTFB       : 2;        // 00 = 0.2256; 01 = 0.1128; 10 = 0.0752; 11 = 0.0564
        };
        uint8_t regValue;
    };
    uint8_t feedback;
    // uint32_t INTFB;
} TPS55289_VOUT_FS_REG;

// Structure for CDC Register (0x05) [reset = 0b11100000]
typedef struct {
    union {
        struct {
            uint8_t SC_MASK     : 1;        // 0 = disabled; 1 = Enabled; Short Circuit Indication
            uint8_t OCP_MASK    : 1;        // 0 = disabled; 1 = Enabled; Over-Current Indication
            uint8_t OVP_MASK    : 1;        // 0 = disabled; 1 = Enabled; Over-Voltage Indication
            uint8_t reserved    : 1;
            uint8_t CDC_OPTION  : 1;        // 0 = Internal; 1 = External; CDC Compensation
            uint8_t CDC         : 3;        // Refer to Table 7-9 of datasheet
        };
        uint8_t regValue;
    };
} TPS55289_CDC_REG;

// Structure for MODE Register (0x06) [reset = 0b00100000]
typedef struct {
    union {
        struct {
            uint8_t OE          : 1;        // 0 = Output Disabled; 1 = Output Enabled
            uint8_t FSWDBL      : 1;        // 0 = Unchanged Freq; 1 = Double Frequency during Buck-Boost Operation
            uint8_t HICCUP      : 1;        // 0 = Disabled; 1 = Enabled; Hiccup Mode
            uint8_t DISCHG      : 1;        // 0 = Disabled; 1 = Enabled; VOUT Discharge in Shutdown Mode
            uint8_t reserved    : 2;
            uint8_t FPWM        : 1;        // 0 = PFM; 1 = FPWM
            uint8_t reserved1   : 1;
        };
        uint8_t regValue;
    };
} TPS55289_MODE_REG;

// Structure for STATUS Register (0x07) [reset = 0b00000011]
typedef struct {
    union {
        struct {
            uint8_t SCP         : 1;        // 0 = No Short Circuit; 1 = Short Circuit Indicator
            uint8_t OCP         : 1;        // 0 = No Overcurrent; 1 = Overcurrent Indicator
            uint8_t OVP         : 1;        // 0 = No OVP; 1 = Over Voltage Indicator
            uint8_t reserved    : 3;
            uint8_t STATUS      : 2;
            /*
                00 = Boost
                01 = Buck
                10 = Buck-Boost
                11 = Reserved
            */
        };
        uint8_t regValue;
    };
} TPS55289_STATUS_REG;

// TPS55289 Device Info Structure
typedef struct {
    // Registers
    TPS55289_REF_VOLTAGE_REG    TPS55289_REF_VOLTAGE;
    TPS55289_IOUT_LIMIT_REG     TPS55289_IOUT_LIMIT;
    TPS55289_VOUT_SR_REG        TPS55289_VOUT_SR;
    TPS55289_VOUT_FS_REG        TPS55289_VOUT_FS;
    TPS55289_CDC_REG            TPS55289_CDC;
    TPS55289_MODE_REG           TPS55289_MODE;
    TPS55289_STATUS_REG         TPS55289_STATUS;

    uint8_t I2C_ADDRESS;
} TPS55289;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function Declarations
_Bool TPS55289Init(TPS55289 *device);
static int setRegister(uint8_t registerAddress, const uint8_t data);
static int getRegister(uint8_t registerAddress, uint8_t data);
_Bool setOutputVoltage(TPS55289 *device, float voltage);
_Bool enableOutputCurrentLimit(TPS55289 *device);
_Bool disableOutputCurrentLimit(TPS55289 *device);
_Bool setOutputCurrentLimit(TPS55289 *device, float currentLimit);
_Bool setOCPResponseTime(TPS55289 *device, uint8_t OCPResponseTime);
_Bool setSlewRate(TPS55289 *device, uint8_t slewRate);
_Bool setFBMechanism(TPS55289 *device, uint8_t FB);
_Bool setStepSize(TPS55289 *device, uint8_t stepSize);
_Bool enableSCIndication(TPS55289 *device);
_Bool disableSCIndication(TPS55289 *device);
_Bool enableOCPIndication(TPS55289 *device);
_Bool disableOCPIndication(TPS55289 *device);
_Bool enableOVPIndication(TPS55289 *device);
_Bool disableOVPIndication(TPS55289 *device);
_Bool setCDCOption(TPS55289 *device, uint8_t CDCOption);
_Bool setCDCComp(TPS55289 *device, int compensation);
_Bool enableDevice(TPS55289 *device);
_Bool disableDevice(TPS55289 *device);
_Bool FSWDoubling(TPS55289 *device, uint8_t input);
_Bool enableHiccupMode(TPS55289 *device);
_Bool disableHiccupMode(TPS55289 *device);
_Bool enableVOUTDSCHG(TPS55289 *device);
_Bool disableVOUTDSCHG(TPS55289 *device);
_Bool FSWOpMode(TPS55289 *device, uint8_t mode);
_Bool readStatusRegister(TPS55289 *device);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // TPS55289_H