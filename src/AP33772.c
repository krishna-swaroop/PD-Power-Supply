// C Library for AP33772 USB-PD Sink Controller from Cypress Semiconductors
/* Structs and Resgister list ported from "AP33772 I2C Command Tester" by Joseph Liang
 * Created 11 April 2022
 */

#include "AP33772.h"
#include "../../pico-sdk/src/rp2_common/hardware_i2c/include/hardware/i2c.h"
// #include "hardware/i2c.h"
#include "pico/stdlib.h"

// Initialisation Function
_Bool AP33772Init(AP33772 *device) {
    // Read Status Register and populate AP33772 Struct
    i2c_read(device, CMD_STATUS, 1);
    // Set Flags accordingly
    device->ap33772_status.readStatus = device->readBuf[0];

    // Set Event Flags
    if(device->ap33772_status.isOvp) {
        device->event_flag.ovp = 1;
    }
    if(device->ap33772_status.isOcp) {
        device->event_flag.ocp = 1;
    }

    if(device->ap33772_status.isReady) {    // negotiation finished
        if (device->ap33772_status.isNewpdo)    // new PDO
        {
            if(device->ap33772_status.isSuccess){
                device->event_flag.newNegoSuccess = 1;
            } else {
                device->event_flag.newNegoFail = 1;
            }
        } else {
            if(device->ap33772_status.isSuccess){
                device->event_flag.negoSuccess = 1;
            } else {
                device->event_flag.negoFail = 1;
            }
        }
    }
    sleep_ms(10);

    // If negotiation is good, load in PDOs from Charger
    if(device->event_flag.newNegoSuccess){
        // Reset new negotiation flag
        device->event_flag.newNegoSuccess = 0;
        
        // CMD: Read PDO Number
        i2c_read(device, CMD_PDONUM, 1);        
        device->numPDO = device->readBuf[0];

        // CMD: Read PDOs
        i2c_read(device, CMD_SRCPDO, SRCPDO_LENGTH);
        // Copy PDOs into pdoData[] Struct
        for (uint8_t i = 0; i < device->numPDO; i++){
            device->pdoData[i].byte0 = device->readBuf[i*4];
            device->pdoData[i].byte1 = device->readBuf[(i*4) + 1];
            device->pdoData[i].byte2 = device->readBuf[(i*4) + 2];
            device->pdoData[i].byte3 = device->readBuf[(i*4) + 3];

            // Check for PPS Capability from Upstream Port
            if((device->pdoData[i].byte3 & 0xF0) == 0xC0){
                // Store Index
                device->PPSindex = i;
                // Turn ON Flag
                device->existPPS = 1;
            }
        }

    }
}

void i2c_read(AP33772 *device, uint8_t cmdAddr, uint8_t len){
    // Clear Read Buffer
    for (uint8_t i = 0; i < READ_BUFF_LENGTH; i++){
        device->readBuf[i] = 0;
    }
    // Send Register Address 
    i2c_write_blocking(i2c0, device->I2C_ADDRESS, &cmdAddr, 1, false);

    // Read data from specified register address
    i2c_read_blocking(i2c0, device->I2C_ADDRESS, &device->readBuf, len, false);
}

void i2c_write(AP33772 *device, uint8_t cmdAddr, uint8_t len){
    // Send register address to write to
    i2c_write_blocking(i2c0, device->I2C_ADDRESS, &cmdAddr, len, false);
    // Send data to specified register address
    i2c_write_blocking(i2c0, device->I2C_ADDRESS, &device->writeBuf[0], len, false);

    // Clear write buffer
    for (uint8_t i = 0; i < WRITE_BUFF_LENGTH; i++){
        device->writeBuf[i] = 0;
    }
}

/**
 *  @brief :Set VBUS Voltage
 *  @param :targetVoltage in mV
*/
void setVoltage(AP33772 *device, int targetVoltage){
    /*
    *   Step-1: Check if PPS can satisfy request voltage
    *   Step-2: Scan PDOs to see what is the lower closest voltage to request
    *   Step-3: Compare found PDOs voltage and PPS max Voltage
    */
        uint8_t tempIndex = 0;
    if((device->existPPS) && (device->pdoData[device->PPSindex].pps.maxVoltage*100 >= targetVoltage) && (device->pdoData[device->PPSindex].pps.minVoltage*100 <= targetVoltage)){
        // Step-1
        device->indexPDO = device->PPSindex;
        device->reqPpsVolt = targetVoltage/20;  // Unit in 20mV/LBS
        device->rdoData.pps.objPosition = device->PPSindex +1; // index 1
        device->rdoData.pps.opCurrent = device->pdoData[device->PPSindex].pps.maxCurrent;
        device->rdoData.pps.voltage = device->reqPpsVolt;
        writeRDO(device);
        return;
    } else {
        // Step-2
        for(uint8_t i = 0; i < (device->numPDO- device->existPPS); i++){
            if(device->pdoData[i].fixed.voltage*50 <= targetVoltage){
                tempIndex = i;
            }
        }
        // Step-3
        if (device->pdoData[tempIndex].fixed.voltage*50 > device->pdoData[device->PPSindex].pps.maxVoltage*100){
            device->indexPDO = tempIndex;
            device->rdoData.fixed.objPosition = tempIndex +1;
            device->rdoData.fixed.maxCurrent = device->pdoData[device->indexPDO].fixed.maxCurrent;
            device->rdoData.fixed.opCurrent = device->pdoData[device->indexPDO].fixed.maxCurrent;
            writeRDO(device);
            return;
        } else {    // if PPS voltage larger or equal to Fixed PDO
            device->indexPDO = device->PPSindex;
            device->reqPpsVolt = device->pdoData[device->PPSindex].pps.maxVoltage*5;
            device->rdoData.pps.objPosition = device->PPSindex + 1;
            device->rdoData.pps.opCurrent = device->pdoData[device->PPSindex].pps.maxCurrent;
            device->rdoData.pps.voltage = device->reqPpsVolt;
            writeRDO(device);
            return;
        }
    }
}

/**
* @brief : Set max current before tripping at wall plug
* @param : targetMaxCurrent in mA
*/
void setMaxCurrent(AP33772 *device, int targetMaxCurrent){
    /*
    *   Step-1: Check if current profile is PPS, check if max current is lower than requested
    *       If yes, set new max current
    *       If no, report fualt
    *   Step-2: If profile is PDO, check if max current is lower than request
    *       If yes, set new max current
    *       if no, report fault
    */
    if(device->indexPDO == device->PPSindex){
        if(targetMaxCurrent <= device->pdoData[device->PPSindex].pps.maxCurrent*50){
            device->rdoData.pps.objPosition = device->PPSindex +1; // index 1
            device->rdoData.pps.opCurrent   = targetMaxCurrent/50;  // 50mA/LBS
            device->rdoData.pps.voltage     = device->reqPpsVolt;
            writeRDO(device);
        } else{}
    } else {
        if(targetMaxCurrent <= device->pdoData[device->indexPDO].fixed.maxCurrent*10){
            device->rdoData.fixed.objPosition   = device->indexPDO +1;  // Index 0 to Index 1
            device->rdoData.fixed.maxCurrent    = targetMaxCurrent/10;  // 10mA.LBS
            device->rdoData.fixed.opCurrent     = targetMaxCurrent/10;  // 10mA/LBS
            writeRDO(device);
        } else{}
    }
}

/**
 *  @brief Set resistance value of 10k NTC at 25C, 50C, 75C and 100C.
 *          Default is 10000, 4161, 1928, 974 Ohms
 *  @param device: Pointer to AP33772 Device
 *  @param TRxx: TR25, TR50, TR75, TR100 unit in Ohm
 *  @attention Blocking Function due to long I2C write, min blocking time 15ms
 */
void setNTC(AP33772 *device, int TR25, int TR50, int TR75, int TR100){
    device->writeBuf[0] = TR25 & 0xFF;
    device->writeBuf[1] = (TR25 >> 8) & 0xFF;
    i2c_write(device, 0x28, 2);
    sleep_ms(5);
    device->writeBuf[0] = TR50 & 0xFF;
    device->writeBuf[1] = (TR50 >> 8) & 0xFF;
    i2c_write(device, 0x2A, 2);
    sleep_ms(5);
    device->writeBuf[0] = TR75 & 0xFF;
    device->writeBuf[1] = (TR75 >> 8) & 0xFF;
    i2c_write(device, 0x2C, 2);
    sleep_ms(5);
    device->writeBuf[0] = TR100 & 0xFF;
    device->writeBuf[1] = (TR100 >> 8) & 0xFF;
    i2c_write(device, 0x2E, 2);
}

/**
 * @brief : Set target temperature (C) when output power through USB-C is reduced
 *          Default is 120C
 * @param : temperature (unit in Celsius)
*/
void setDeratingTemp(AP33772 *device, int temperature){
    device->writeBuf[0] = temperature;
    i2c_write(device, CMD_DRTHR, 1);
}

/**
 * @brief : Set Over Current Protection Threshold
 *          DEfault is 0mA. Remember to change this before enable OCP_EN Flag
 * @param : OCP_current (unit in mA)
*/
void setOCPTHR(AP33772 *device, int OCP_current){
    device->writeBuf[0] = OCP_current/50; //50mA LSB
    i2c_write(device, CMD_OCPTHR, 1);
}

/**
 * @brief : Set Over Temperature Protection Threshold
 *          Default is 120C. Remember to set OTP_EN Flag
 * @param : OTP_temperature (unit in C)
*/
void setOTPTHR(AP33772 *device, int OTP_temperature)
{
    device->writeBuf[0] = OTP_temperature;
    i2c_write(device, CMD_OTPTHR, 1);
}

/**
 * @brief : Set MASK Function
 * @param : Flag ID
*/
void setMask(AP33772 *device,AP33772_MASK flag){
    // Read current flag setting
    i2c_read(device, CMD_MASK, 1);
    device->writeBuf[0] = device->readBuf[0] | flag;
    sleep_ms(5);
    i2c_write(device, CMD_MASK, 1);
}

/**
 * @brief : Clear MASK Function
 * @param : Flag ID
*/
void clearMask(AP33772 *device, AP33772_MASK flag){
    // Read current flag setting
    i2c_read(device, CMD_MASK, 1);
    device->writeBuf[0] = device->readBuf[0] & ~flag;
    sleep_ms(5);
    i2c_write(device, CMD_MASK, 1);
}

/**
 * @brief : Write the desired power profile back to power source
*/
void writeRDO(AP33772 *device){
    device->writeBuf[3] = device->rdoData.byte3;
    device->writeBuf[2] = device->rdoData.byte2;
    device->writeBuf[1] = device->rdoData.byte1;
    device->writeBuf[0] = device->rdoData.byte0;
    // Write to RDO Register
    i2c_write(device, CMD_RDO, 4);
}

/**
 * @brief : Read VBUS Voltage
 * @return: Voltage in mV
*/
int readVoltage(AP33772 *device){
    i2c_read(device, CMD_VOLTAGE, 1);
    return device->readBuf[0]* 80;      // 80mV/LSB
}

/**
 * @brief: Read VBUS Current
 * @return: Current in mA
*/
int readCurrent(AP33772 *device){
    i2c_Read(device, CMD_CURRENT, 1);
    return device->readBuf[0] * 16;     // 24mA/LSB
}

/**
 * @brief: Read maximum VBUS Current
 * @return: Current in mA
*/
int getMaxCurrent(AP33772 *device){
    if(device->indexPDO == device->PPSindex){
        return device->pdoData[device->PPSindex].pps.maxCurrent*50;
    } else {
        return device->pdoData[device->indexPDO].fixed.maxCurrent*10;
    }
}

/**
 * @brief Read NTC temperature
 * @return tempearture in C
 */
int readTemp(AP33772 *device){
    i2c_read(device, CMD_TEMP, 1);
    return device->readBuf[0];      // 1C LSB
}

/**
 * @brief Debug Code to quickly check power supply profile PDOs
 *          Dumps all PDOs profile into Serial Port
*/
void printPDO(AP33772 *device){
    printf("Source PDO Number = ");
    printf("%d",device->numPDO);
}

/**
 * @brief : Hard Reset the Power Supply. Will temporarily cause power outage
*/
void reset(AP33772 *device){
    device->writeBuf[0] = 0x00;
    device->writeBuf[1] = 0x00;
    device->writeBuf[2] = 0x00;
    device->writeBuf[3] = 0x00;
    i2c_write(device, CMD_RDO, 4);
}
