#include "MPL3115A2.h"

#include "i2c_wrapper.hpp"

namespace MPL{

    const uint8_t I2C_ADDR = 0x60; //APPARENTLY I2C Library doesn't want shifted address

    const uint8_t REG_STATUS = 0x00;
    const uint8_t REG_OUT_P = 0x01;
    const uint8_t REG_OUT_T = 0x04;
    const uint8_t REG_SYSMOD = 0x11;
    const uint8_t REG_PT_DATA_CFG = 0x13;
    const uint8_t REG_CTRL_REG1 = 0x26;
    const uint8_t REG_CTRL_REG3 = 0x29;
    const uint8_t REG_CTRL_REG4 = 0x29;
    const uint8_t REG_CTRL_REG5 = 0x2A;
    const uint8_t REG_OFF_H = 0x2D;
    const uint8_t BASE_CTRL = 0xA0; //Altimeter mode, 16x oversample (66ms/samp)
    const uint8_t ACQ_REQUEST = 0x02; //OR mask to add immediate measurement bit

    AltTempData decode(uint8_t* raw){
        AltTempData ret;
        ret.alt = ((uint32_t)raw[2]<<8)|((uint32_t)raw[1]<<16)|((uint32_t)raw[0]<<24);
        ret.alt = ret.alt >> 12; //Shift back with sign extend
        ret.temp = raw[4]|((uint16_t)raw[3]<<8);
        ret.temp = ret.temp >> 4; //Shift back with sign extend
        return ret;
    }

    AltTempData ReadData() {
        uint8_t raw[5];
        i2c_read(I2C_ADDR, REG_OUT_P, 5, raw);
        return decode(raw);
    }

    bool CheckAndRead(AltTempData* result) {
        uint8_t data[6];
        i2c_read(I2C_ADDR, REG_STATUS, 6, data); // Status and data (maybe)
        if(data[0] & 0x08){
            *result = decode(data+1);
            return true;
        } else {
            return false;
        }
    }

    AltTempData BlockingRead() {
        uint8_t status;
        do{
            i2c_read(I2C_ADDR, REG_STATUS, 1, &status);
        } while(!(status & 0x08));
        return ReadData();
    }

    bool IsDataReady() {
        uint8_t status;
        i2c_read(I2C_ADDR, REG_STATUS, 1, &status);
        return status & 0x08;
    }

    void SetStandby(bool stby) {
        i2c_write(I2C_ADDR, REG_SYSMOD, (uint8_t)(stby ? 0x00 : 0x01));
    }

    void RequestData() {
        i2c_write(I2C_ADDR, REG_CTRL_REG1, (uint8_t)(BASE_CTRL | ACQ_REQUEST));
    }

    void Init() {
        i2c_write(I2C_ADDR, REG_PT_DATA_CFG, (uint8_t)0x07); //Enable data ready flags
        i2c_write(I2C_ADDR, REG_CTRL_REG1, BASE_CTRL); //Base config
    }

    bool CheckDevicePresent() { //Check WHO_AM_I register
        uint8_t whoami;
        i2c_read(I2C_ADDR, 0x0C, 1, &whoami);
        return whoami == 0xC4;  
    }

}