#include "pico/stdlib.h"
// #include "pico/binary_info.h"
// #include "/home/swaroop/pico/pico-sdk/src/rp2_common/hardware_i2c/include/hardware/i2c.h"
#include "../../pico-sdk/src/rp2_common/hardware_i2c/include/hardware/i2c.h"

#include "TPS55289.h"
#include "math.h"
#include <stdio.h>

/*
    Initialisation Function
*/ 
// extern TPS55289 device;

_Bool TPS55289Init(TPS55289 *device){
    _Bool STATUS = true;
    
    uint8_t TPS55289_REF_VOLTAGE_LSB_DEFVAL = 0b00000000;
    uint8_t TPS55289_REF_VOLTAGE_MSB_DEFVAL = 0b00000000;
    uint8_t TPS55289_IOUT_LIMIT_DEFVAL      = 0b11100100;
    uint8_t TPS55289_VOUT_SR_DEFVAL         = 0b00000001;
    uint8_t TPS55289_VOUT_FS_DEFVAL         = 0b10000011;
    uint8_t TPS55289_CDC_DEFVAL             = 0b11100000;
    uint8_t TPS55289_MODE_DEFVAL            = 0b00100000;    
    uint8_t TPS55289_STATUS_DEFVAL          = 0b00000011;
    
    if(!disableDevice(device)){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }

    // Set Register Structures to default values
    device->TPS55289_REF_VOLTAGE.regValue_16    = (TPS55289_REF_VOLTAGE_MSB_DEFVAL << 8)|(TPS55289_REF_VOLTAGE_LSB_DEFVAL);
    device->TPS55289_IOUT_LIMIT.regValue        = TPS55289_IOUT_LIMIT_DEFVAL;
    device->TPS55289_VOUT_SR.regValue           = TPS55289_VOUT_SR_DEFVAL;
    device->TPS55289_VOUT_FS.regValue           = TPS55289_VOUT_FS_DEFVAL;
    device->TPS55289_CDC.regValue               = TPS55289_CDC_DEFVAL;
    device->TPS55289_MODE.regValue              = TPS55289_MODE_DEFVAL;
    device->TPS55289_STATUS.regValue            = TPS55289_STATUS_DEFVAL;

    // Update Registers in the device
    if(setRegister(TPS55289_REF_VOLTAGE_LSB_ADDR,TPS55289_REF_VOLTAGE_LSB_DEFVAL) != 1){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }
    if(setRegister(TPS55289_REF_VOLTAGE_MSB_ADDR,TPS55289_REF_VOLTAGE_MSB_DEFVAL) != 1){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }
    if(setRegister(TPS55289_IOUT_LIMIT_ADDR,TPS55289_IOUT_LIMIT_DEFVAL) != 1){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }
    if(setRegister(TPS55289_VOUT_SR_ADDR,TPS55289_VOUT_SR_DEFVAL) != 1){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }
    if(setRegister(TPS55289_VOUT_FS_ADDR,TPS55289_VOUT_FS_DEFVAL) != 1){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }
    if(setRegister(TPS55289_CDC_ADDR,TPS55289_CDC_DEFVAL) != 1){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }
    if(setRegister(TPS55289_MODE_ADDR,TPS55289_MODE_DEFVAL) != 1){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }
    if(setRegister(TPS55289_CDC_ADDR,TPS55289_CDC_DEFVAL) != 1){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }
    // Enable Device after setting all registers with default values
    if(!enableDevice(device)){
        printf("Failed to initialise TPS55289\n");
        STATUS = false;
        return STATUS;
    }
    STATUS = readStatusRegister(device);

    return STATUS;
}

/*
    Set Register Function
*/
static int setRegister(uint8_t registerAddress, const uint8_t data) {
    uint8_t buffer[2];
    buffer[0] = registerAddress;
    buffer[1] = data;

    return i2c_write_blocking(i2c0, TPS55289_I2C_ADDR, &buffer[0], 2, false);
}
/*
    Get Register Function
*/
static int getRegister(uint8_t registerAddress, uint8_t data) {
    if (i2c_write_blocking(i2c0, TPS55289_I2C_ADDR, &registerAddress, 1, false) != 1) {
        return false; // Error writing register address
    }
    return i2c_read_blocking(i2c0, TPS55289_I2C_ADDR, &data, 1, false);
}

_Bool setOutputVoltage(TPS55289 *device, float voltage){
    _Bool STATUS = true;
    // Check if the voltage requested is valid
    if (((voltage >= 0.8) && (voltage <= 22)) == 0)
    {
        printf("Requested Output Voltage is invalid");
        STATUS = false;
        return STATUS;
    }
    // Disabling the output before changing parameters
    printf("Disabling Output\n");
    if(disableDevice(device) != true){
        printf("Failed to Disable Output\n");
        STATUS = false;
        return STATUS;
    }
    printf("Disabled Output\n");
    device->TPS55289_REF_VOLTAGE.VOUT = voltage;
    float referenceVoltage = voltage*device->TPS55289_REF_VOLTAGE.CURRENT_INTFB; // in Volts
    device->TPS55289_REF_VOLTAGE.regValue_16 = (uint16_t)(1.7715*((referenceVoltage*1000) - 45)+1); // Each step is 0.5645mV. 0x000 starts at 45mV

    // Update local register values with new reference voltage
    device->TPS55289_REF_VOLTAGE.VREF_LSB = device->TPS55289_REF_VOLTAGE.regValue_16 & 0xFF;
    device->TPS55289_REF_VOLTAGE.VREF_MSB = (device->TPS55289_REF_VOLTAGE.regValue_16>>8) & 0xFF;

    // Update registers on device
    if(setRegister(TPS55289_REF_VOLTAGE_LSB_ADDR, (device->TPS55289_REF_VOLTAGE.VREF_LSB)) != 1){
        STATUS = false;
        return false;
    }
    if(setRegister(TPS55289_REF_VOLTAGE_MSB_ADDR, device->TPS55289_REF_VOLTAGE.VREF_MSB) != 1){
        STATUS = false;
        return false;
    }

    printf("Voltage Set: %f mV\n", voltage);
    printf("Enabling Output\n");
    if(enableDevice(device) != true){
        printf("Failed to enable Output\n");
        STATUS = false;
        return STATUS;
    }
    printf("Enabled Output\n");

    return STATUS;
}

_Bool enableOutputCurrentLimit(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_IOUT_LIMIT.Current_Limit_EN = 0b1;
    if (setRegister(TPS55289_IOUT_LIMIT_ADDR,device->TPS55289_IOUT_LIMIT.regValue) != 1)
    {
        printf("Couldn't Enable Current Limit\n");
        STATUS = false;
        return STATUS;
    }
    printf("Enabled Output Current Limit\n");
    printf("Output Current Limit = %u A", device->TPS55289_IOUT_LIMIT.currentLimitAmp);   
    return STATUS;
}

_Bool disableOutputCurrentLimit(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_IOUT_LIMIT.Current_Limit_EN = 0b0;
    if (setRegister(TPS55289_IOUT_LIMIT_ADDR,device->TPS55289_IOUT_LIMIT.regValue) != 1)
    {
        printf("Couldn't Disable Current Limit\n");
        STATUS = false;
        return STATUS;
    }
    printf("Disabled Output Current Limit\n");   
    return STATUS;
}

_Bool setOutputCurrentLimit(TPS55289 *device, float currentLimit){
    _Bool STATUS = true;
    // Check if requested current limit is valid
    if((fmod(currentLimit, 0.05) != 0)|| (currentLimit >= 0.0) || (currentLimit <= 6.35)){
        printf("Invalid Current Limit Selected\n");
        printf("Current Limit needs to be between 0.0 and 6.35 and must be a mmultiple of 0.05A\n");;
        STATUS = false;
        return STATUS;
    }
    device->TPS55289_IOUT_LIMIT.currentLimitAmp = currentLimit;
    float Vdiff = (uint8_t)currentLimit*TPPS55289_SENSE_RESISTOR;       // This will give Vdiff in mV
    device->TPS55289_IOUT_LIMIT.Current_Limit_Setting = Vdiff/(0.5);    // Step size is 0.5mV
    if (setRegister(TPS55289_IOUT_LIMIT_ADDR,device->TPS55289_IOUT_LIMIT.regValue) != 1)
    {
        printf("Couldn't Set Ouput Current Limit\n");
        STATUS = false;
        return STATUS;
    }
    printf("Output Current Limit Set Succesfully!\n");
    printf("Output Current Limit: %f A\n", currentLimit);
    return STATUS;
}

_Bool setOCPResponseTime(TPS55289 *device, uint8_t OCPResponseTime){
    _Bool STATUS = true;
    switch (OCPResponseTime)
    {
    case 0x00:
        device->TPS55289_VOUT_SR.OCP_DELAY = 0b00;
        device->TPS55289_VOUT_SR.OCResponseTime = 0.128;    // in milliseconds
        break;
    case 0x01:
        device->TPS55289_VOUT_SR.OCP_DELAY = 0b01;
        device->TPS55289_VOUT_SR.OCResponseTime = 1.024*3;    // in milliseconds
        break;
    case 0x02:
        device->TPS55289_VOUT_SR.OCP_DELAY = 0b10;
        device->TPS55289_VOUT_SR.OCResponseTime = 1.024*6;    // in milliseconds
        break;
    case 0x03:
        device->TPS55289_VOUT_SR.OCP_DELAY = 0b11;
        device->TPS55289_VOUT_SR.OCResponseTime = 1.024*12;    // in milliseconds
        break;
    default:
        printf("Invalid Response Time Selected\n");
        printf("Valid Response Time inputs are 0x00-0x03\n");
        break;
    }
    if (setRegister(TPS55289_VOUT_SR_ADDR,device->TPS55289_VOUT_SR.regValue) != 1)
    {
        printf("Couldn't Set Overcurrent Protection Response Time\n");
        STATUS = false;
        return STATUS;
    }
    return STATUS;
}

_Bool setSlewRate(TPS55289 *device, uint8_t slewRate){
    _Bool STATUS = true;
    switch (slewRate)
    {
    case 0x00:
        device->TPS55289_VOUT_SR.SR = 0b00;
        device->TPS55289_VOUT_SR.slewRate = 1.25;       // in mV/us
        break;
    case 0x01:
        device->TPS55289_VOUT_SR.SR = 0b01;
        device->TPS55289_VOUT_SR.slewRate = 2.5;
        break;
    case 0x02:
        device->TPS55289_VOUT_SR.SR = 0b10;
        device->TPS55289_VOUT_SR.slewRate = 5.0;
        break;
    case 0x03:
        device->TPS55289_VOUT_SR.SR = 0b11;
        device->TPS55289_VOUT_SR.slewRate = 10.0;
        break;
    default:
        printf("Invalid Slew Rates Selected\n");
        printf("Valid Slew Rate inputs are 0x00-0x03\n");
        break;
    }
    if (setRegister(TPS55289_VOUT_SR_ADDR,device->TPS55289_VOUT_SR.regValue) != 1)
    {
        printf("Couldn't Set Output Voltage Slew Rate\n");
        STATUS = false;
        return STATUS;
    }
    return STATUS;
}

_Bool setFBMechanism(TPS55289 *device, uint8_t FB){
    _Bool STATUS = true;
    if(FB == 0){
        device->TPS55289_VOUT_FS.FB = 0;
        printf("Feedback Mechanism set to Internal Feedback\n");
    } else {
        device->TPS55289_VOUT_FS.FB = 1;
        printf("Feedback Mechanism set to External Feedback\n");
    }
    if (setRegister(TPS55289_VOUT_FS_ADDR,device->TPS55289_VOUT_FS.regValue) != 1)
    {
        printf("Couldn't Set Updated Feedback Mechanism\n");
        STATUS = false;
        return STATUS;
    }
    return STATUS;
}

_Bool setStepSize(TPS55289 *device, uint8_t stepSize){
    _Bool STATUS = true;
    switch (stepSize)
    {
    case 0x00:          // 2.5mV Step Size
        device->TPS55289_VOUT_FS.INTFB = 0b00;
        device->TPS55289_REF_VOLTAGE.CURRENT_INTFB = 0.2256;
        printf("Output Voltage Step Size: 2.5mV\n");
        break;
    case 0x01:          // 5mV Step Size
        device->TPS55289_VOUT_FS.INTFB = 0b01;
        device->TPS55289_REF_VOLTAGE.CURRENT_INTFB = 0.1128;
        printf("Output Voltage Step Size: 5mV\n");
        break;
    case 0x02:          // 7.5mV Step Size
        device->TPS55289_VOUT_FS.INTFB = 0b10;
        device->TPS55289_REF_VOLTAGE.CURRENT_INTFB = 0.0752;
        printf("Output Voltage Step Size: 7.5mV\n");
        break;
    case 0x03:          // 10mV Step Size
        device->TPS55289_VOUT_FS.INTFB = 0b11;
        device->TPS55289_REF_VOLTAGE.CURRENT_INTFB = 0.0564;
        printf("Output Voltage Step Size: 10mV\n");
        break;
    
    default:
        printf("Invalid Step Size Requested\n");
        printf("Valid Step Sizes are 0x00-0x03\n");
        break;
    }
    if (setRegister(TPS55289_VOUT_FS_ADDR,device->TPS55289_VOUT_FS.regValue) != 1)
    {
        printf("Couldn't Update Output Voltage Step Size\n");
        STATUS = false;
        return STATUS;
    }
    return STATUS;
}

_Bool enableSCIndication(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_CDC.SC_MASK = 0b1;
    if (setRegister(TPS55289_CDC_ADDR,device->TPS55289_CDC.regValue) != 1)
    {
        printf("Couldn't Enable Short Circuit Indication\n");
        STATUS = false;
        return STATUS;
    }
    printf("Enabled Short Circuit Indication\n");
    return STATUS;
}

_Bool disableSCIndication(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_CDC.SC_MASK = 0b0;
    if (setRegister(TPS55289_CDC_ADDR,device->TPS55289_CDC.regValue) != 1)
    {
        printf("Couldn't Disable Short Circuit Indication\n");
        STATUS = false;
        return STATUS;
    }
    printf("Disabled Short Circuit Indication\n");
    return STATUS;
}

_Bool enableOCPIndication(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_CDC.OCP_MASK = 0b1;
    if (setRegister(TPS55289_CDC_ADDR,device->TPS55289_CDC.regValue) != 1)
    {
        printf("Couldn't Enable OCP Indication\n");
        STATUS = false;
        return STATUS;
    }
    printf("Enabled OCP Indication\n");
    return STATUS;
}

_Bool disableOCPIndication(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_CDC.OCP_MASK = 0b0;
    if (setRegister(TPS55289_CDC_ADDR,device->TPS55289_CDC.regValue) != 1)
    {
        printf("Couldn't Disable OCP Indication\n");
        STATUS = false;
        return STATUS;
    }
    printf("Disabled OCP Indication\n");
    return STATUS;
}

_Bool enableOVPIndication(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_CDC.OVP_MASK = 0b1;
    if (setRegister(TPS55289_CDC_ADDR,device->TPS55289_CDC.regValue) != 1)
    {
        printf("Couldn't Enable OVP Indication\n");
        STATUS = false;
        return STATUS;
    }
    printf("Enabled OVP Indication\n");
    return STATUS;
}

_Bool disableOVPIndication(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_CDC.OVP_MASK = 0b0;
    if (setRegister(TPS55289_CDC_ADDR,device->TPS55289_CDC.regValue) != 1)
    {
        printf("Couldn't Disable OVP Indication\n");
        STATUS = false;
        return STATUS;
    }
    printf("Disabled OVP Indication\n");
    return STATUS;
}

_Bool setCDCOption(TPS55289 *device, uint8_t CDCOption){
    _Bool STATUS = true;
    if(CDCOption == 0){
        device->TPS55289_CDC.CDC_OPTION = 0b0;
    } else {
        device->TPS55289_CDC.CDC_OPTION = 0b1;
    }
    if (setRegister(TPS55289_CDC_ADDR,device->TPS55289_CDC.regValue) != 1)
    {
        printf("Couldn't set CDC Option\n");
        STATUS = false;
        return STATUS;
    }
    if (CDCOption == 0)
    {
        printf("Internal CDC Compensation Set\n");
    } else {
        printf("External CDC Compensation Set\n");
    }
    return STATUS;
}

_Bool setCDCComp(TPS55289 *device, int compensation){
    _Bool STATUS = true;
    switch (compensation)
    {
    case 0x00:
        device->TPS55289_CDC.CDC = 0b000;
        printf("Compensation set at 0V");
        break;
    case 0x01:
        device->TPS55289_CDC.CDC = 0b001;
        printf("Compensation set at 0.1V");
        break;
    case 0x02:
        device->TPS55289_CDC.CDC = 0b010;
        printf("Compensation set at 0.2V");
        break;
    case 0x03:
        device->TPS55289_CDC.CDC = 0b011;
        printf("Compensation set at 0.3V");
        break;
    case 0x04:
        device->TPS55289_CDC.CDC = 0b100;
        printf("Compensation set at 0.4V");
        break;
    case 0x05:
        device->TPS55289_CDC.CDC = 0b101;
        printf("Compensation set at 0.5V");
        break;
    case 0x06:
        device->TPS55289_CDC.CDC = 0b110;
        printf("Compensation set at 0.6V");
        break;
    case 0x07:
        device->TPS55289_CDC.CDC = 0b111;
        printf("Compensation set at 0.7V");
        break;
    default:
        printf("Invalid Compensation Requested\n");
        printf("Valid Compensation Presets are 0x00-0x07\n");
        break;
    }
    if (setRegister(TPS55289_CDC_ADDR,device->TPS55289_CDC.regValue) != 1)
    {
        printf("Couldn't set CDC Compensation\n");
        STATUS = false;
        return STATUS;
    }
    return STATUS;
}

_Bool enableDevice(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_MODE.OE = 0b1;     // Enable device
    if (setRegister(TPS55289_MODE_ADDR,device->TPS55289_MODE.regValue) != 1)
    {
        printf("Couldn't Enable Device\n");
        STATUS = false;
        return STATUS;
    }
    printf("Enabled Device\n");
    return STATUS;
}

_Bool disableDevice(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_MODE.OE = 0b0;     // Enable device
    if (setRegister(TPS55289_MODE_ADDR,device->TPS55289_MODE.regValue) != 1)
    {
        printf("Couldn't Disable Device\n");
        STATUS = false;
        return STATUS;
    }
    printf("Disabled Device\n");
    return STATUS;
}


_Bool FSWDoubling(TPS55289 *device, uint8_t input){
    _Bool STATUS = true;
    if (input == 1){
        device->TPS55289_MODE.FSWDBL = 0b1;     // Double Freq in Buck-Boost Operating Mode    
    } else {
        device->TPS55289_MODE.FSWDBL = 0b1;     // Keepp same Freq in Buck-Boost Operating Mode
    }
    
    if (setRegister(TPS55289_MODE_ADDR,device->TPS55289_MODE.regValue) != 1)
    {
        printf("Couldn't set FSWDBL Mode\n");
        STATUS = false;
        return STATUS;
    }
    printf("Set FSWDBL Mode Successfully\n");
    return STATUS;
}

_Bool enableHiccupMode(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_MODE.HICCUP = 0b1;     // Enable Hiccup Mode
    if (setRegister(TPS55289_MODE_ADDR,device->TPS55289_MODE.regValue) != 1)
    {
        printf("Couldn't Enable Hiccup Mode\n");
        STATUS = false;
        return STATUS;
    }
    printf("Enabled Hiccup Mode\n");
    return STATUS;
}

_Bool disableHiccupMode(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_MODE.HICCUP = 0b1;     // Disable Hiccup Mode
    if (setRegister(TPS55289_MODE_ADDR,device->TPS55289_MODE.regValue) != 1)
    {
        printf("Couldn't Disable Hiccup Mode\n");
        STATUS = false;
        return STATUS;
    }
    printf("Disabled Hiccup Mode\n");
    return STATUS;
}

_Bool enableVOUTDSCHG(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_MODE.DISCHG = 0b1;     // Enable VOUT Discharge Functionality
    if (setRegister(TPS55289_MODE_ADDR,device->TPS55289_MODE.regValue) != 1)
    {
        printf("Couldn't Enable Discharge Mode\n");
        STATUS = false;
        return STATUS;
    }
    printf("Enabled Discharge Mode\n");
    return STATUS;
}

_Bool disableVOUTDSCHG(TPS55289 *device){
    _Bool STATUS = true;
    device->TPS55289_MODE.DISCHG = 0b0;     // Enable VOUT Discharge Functionality
    if (setRegister(TPS55289_MODE_ADDR,device->TPS55289_MODE.regValue) != 1)
    {
        printf("Couldn't Disable Discharge Mode\n");
        STATUS = false;
        return STATUS;
    }
    printf("Disabled Discharge Mode\n");
    return STATUS;
}

_Bool FSWOpMode(TPS55289 *device, uint8_t mode){
    _Bool STATUS = true;
    if(mode == 0){
        device->TPS55289_MODE.FPWM = 0b0;     // Enable PFM Operating Mode
    } else {
        device->TPS55289_MODE.FPWM = 0b1;     // Enable FPWM Operating Mode
    }
    
    if (setRegister(TPS55289_MODE_ADDR,device->TPS55289_MODE.regValue) != 1)
    {
        printf("Couldn't set Light Load Operating Mode\n");
        STATUS = false;
        return STATUS;
    }
    printf("Set Light Load Operating Mode Successfully\n");
    return STATUS;    
}

_Bool readStatusRegister(TPS55289 *device){
    _Bool STATUS = true;
    if(getRegister(TPS55289_STATUS_ADDR, device->TPS55289_STATUS.regValue) != 1){
        printf("Failed to read Status Register\n");
        STATUS = false;
        return STATUS;
    }
    return STATUS; 
}

_Bool operateOnStatusRegister(TPS55289 *device){
    _Bool STATUS = true;
    STATUS = readStatusRegister(device);
    if(device->TPS55289_STATUS.SCP == 1){
        STATUS = disableDevice(device);
        printf("Short Circuit Condition Detected\n");
        printf("Disabled Output Voltage\n");
        /*
            Add code to send info back to PC GUI
        */
    }
    if(device->TPS55289_STATUS.OCP == 1){
        STATUS = disableDevice(device);
        printf("Overcurrent Condition Detected\n");
        printf("Disabled Output Voltage\n");
        /*
            Add code to send info back to PC GUI
        */
    }
    if(device->TPS55289_STATUS.OVP == 1){
        STATUS = disableDevice(device);
        printf("Overcurrent Condition Detected\n");
        printf("Disabled Output Voltage\n");
        /*
            Add code to send info back to PC GUI
        */
    }
    return STATUS;
}