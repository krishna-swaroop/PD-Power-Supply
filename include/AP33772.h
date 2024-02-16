// C Library for Interfacing the AP33772 USB-PD Controller IC
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

/* Structs and Resgister list ported from "AP33772 I2C Command Tester" by Joseph Liang
 * Created 11 April 2022
 * Added class and class functions by VicentN for PicoPD evaluation board
 */


#ifndef __AP33772__
#define __AP33772__

#include "pico/stdlib.h"

#define CMD_SRCPDO 0x00
#define CMD_PDONUM 0x1C
#define CMD_STATUS 0x1D
#define CMD_MASK 0x1E
#define CMD_VOLTAGE 0x20
#define CMD_CURRENT 0x21
#define CMD_TEMP 0x22
#define CMD_OCPTHR 0x23
#define CMD_OTPTHR 0x24
#define CMD_DRTHR 0x25
#define CMD_RDO 0x30

#define AP33772_ADDRESS 0x51
#define READ_BUFF_LENGTH     30
#define WRITE_BUFF_LENGTH    6
#define SRCPDO_LENGTH        28

typedef enum{
  READY_EN    = 1 << 0, // 0000 0001
  SUCCESS_EN  = 1 << 1, // 0000 0010
  NEWPDO_EN   = 1 << 2, // 0000 0100
  OVP_EN      = 1 << 4, // 0001 0000
  OCP_EN      = 1 << 5, // 0010 0000
  OTP_EN      = 1 << 6, // 0100 0000
  DR_EN       = 1 << 7  // 1000 0000
} AP33772_MASK;

typedef struct
{
  union
  {
    struct
    {
      uint8_t isReady : 1;
      uint8_t isSuccess : 1;
      uint8_t isNewpdo : 1;
      uint8_t reserved : 1;
      uint8_t isOvp : 1;
      uint8_t isOcp : 1;
      uint8_t isOtp : 1;
      uint8_t isDr : 1;
    };
    uint8_t readStatus;
  };
  uint8_t readVolt; // LSB: 80mV
  uint8_t readCurr; // LSB: 24mA
  uint8_t readTemp; // unit: 1C
} AP33772_STATUS_T_STRUCT;

typedef struct
{
  union
  {
    struct
    {
      uint8_t newNegoSuccess : 1;
      uint8_t newNegoFail : 1;
      uint8_t negoSuccess : 1;
      uint8_t negoFail : 1;
      uint8_t reserved_1 : 4;
    };
    uint8_t negoEvent;
  };
  union
  {
    struct
    {
      uint8_t ovp : 1;
      uint8_t ocp : 1;
      uint8_t otp : 1;
      uint8_t dr : 1;
      uint8_t reserved_2 : 4;
    };
    uint8_t protectEvent;
  };
} EVENT_FLAG_T_STRUCT;

typedef struct
{
  union
  {
    struct
    {
      unsigned int maxCurrent : 10; // unit: 10mA
      unsigned int voltage : 10;    // unit: 50mV
      unsigned int reserved_1 : 10;
      unsigned int type : 2;
    } fixed;
    struct
    {
      unsigned int maxCurrent : 7; // unit: 50mA
      unsigned int reserved_1 : 1;
      unsigned int minVoltage : 8; // unit: 100mV
      unsigned int reserved_2 : 1;
      unsigned int maxVoltage : 8; // unit: 100mV
      unsigned int reserved_3 : 3;
      unsigned int apdo : 2;
      unsigned int type : 2;
    } pps;
    struct
    {
      uint8_t byte0;
      uint8_t byte1;
      uint8_t byte2;
      uint8_t byte3;
    };
    unsigned long data;
  };
} PDO_DATA_T_STRUCT;

typedef struct
{
  union
  {
    struct
    {
      unsigned int maxCurrent : 10; // unit: 10mA
      unsigned int opCurrent : 10;  // unit: 10mA
      unsigned int reserved_1 : 8;
      unsigned int objPosition : 3;
      unsigned int reserved_2 : 1;
    } fixed;
    struct
    {
      unsigned int opCurrent : 7; // unit: 50mA
      unsigned int reserved_1 : 2;
      unsigned int voltage : 11; // unit: 20mV
      unsigned int reserved_2 : 8;
      unsigned int objPosition : 3;
      unsigned int reserved_3 : 1;
    } pps;
    struct
    {
      uint8_t byte0;
      uint8_t byte1;
      uint8_t byte2;
      uint8_t byte3;
    };
    unsigned long data;
  };
} RDO_DATA_T_STRUCT;

typedef struct {
    // Data Structures
    AP33772_STATUS_T_STRUCT ap33772_status;
    EVENT_FLAG_T_STRUCT     event_flag;
    PDO_DATA_T_STRUCT       pdoData[7];
    RDO_DATA_T_STRUCT       rdoData;

    uint8_t                 I2C_ADDRESS; 
    uint8_t                 readBuf[READ_BUFF_LENGTH];
    uint8_t                 writeBuf[WRITE_BUFF_LENGTH];
    uint8_t                 numPDO;
    uint8_t                 indexPDO;
    int                     reqPpsVolt;
    uint8_t                 existPPS;
    int8_t                  PPSindex;

} AP33772;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function Declarations
_Bool AP33772Init(AP33772 *device);
// static int setAP33772Register(uint8_t registerAddress, const uint8_t data);
// static int setAP33772Register(uint8_t registerAddress, uint8_t data);
void setVoltage(AP33772 *device, int targetVoltage);     // Unit in mV
void setMaxCurrent(AP33772 *device, int targetMaxCurrent); // Uni t in mA
void setNTC(AP33772 *device, int TR25, int TR50, int TR75, int TR100);
void setDeratingTemp(AP33772 *device, int temperature);
void setOCPTHR(AP33772 *device, int OCP_current);
void setOTPTHR(AP33772 *device, int OTP_temperature);
void setMask(AP33772 *device, AP33772_MASK flag);
void clearMask(AP33772 *device,AP33772_MASK flag);
void writeRDO(AP33772 *device);
int readVoltage(AP33772 *device);
int readCurrent(AP33772 *device);
int getMaxCurrent(AP33772 *device);
int readTemp(AP33772 *device);
void printPDO(AP33772 *device);
void reset(AP33772 *device);
void i2c_read(AP33772 *device, uint8_t cmdAddr, uint8_t len);
void i2c_write(AP33772 *device, uint8_t cmdAddr, uint8_t len);

#endif