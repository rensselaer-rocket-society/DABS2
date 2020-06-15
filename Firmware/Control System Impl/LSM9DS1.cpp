#include "LSM9DS1.h"
#include "i2c_wrapper.hpp"

namespace LSM{

    const uint8_t I2C_IMU = 0x6B;
    const uint8_t I2C_MAG = 0x1E;

    const uint8_t REG_WHO_AM_I = 0x0F;

    // IMU Registers
    const uint8_t REG_STATUS_IMU = 0x17;
    const uint8_t REG_CTRL_REG1_G = 0x10;
    const uint8_t REG_CTRL_REG2_G = 0x11;
    const uint8_t REG_CTRL_REG3_G = 0x12;
    const uint8_t REG_OUT_G = 0x18;
    const uint8_t REG_CTRL_REG5_XL = 0x1F;
    const uint8_t REG_CTRL_REG6_XL = 0x20;
    const uint8_t REG_CTRL_REG7_XL = 0x21;
    const uint8_t REG_CTRL_REG8 = 0x21;
    const uint8_t REG_OUT_XL = 0x28;


    // Magnetometer Registers
    const uint8_t REG_CTRL_REG1_M = 0x20;
    const uint8_t REG_CTRL_REG2_M = 0x21;
    const uint8_t REG_CTRL_REG3_M = 0x22;
    const uint8_t REG_CTRL_REG4_M = 0x23;
    const uint8_t REG_CTRL_REG5_M = 0x24;
    const uint8_t REG_STATUS_M = 0x27;
    const uint8_t REG_OUT_M = 0x28;


    IMUData ReadIMU()
    {
        IMUData dat;
        i2c_read(I2C_IMU, REG_OUT_XL, 6, (uint8_t*)&dat.accel);
        i2c_read(I2C_IMU, REG_OUT_G, 6, (uint8_t*)&dat.gyro);
        return dat;
    }

    vec_i16_t ReadMag()
    {
        vec_i16_t dat;
        i2c_read(I2C_MAG, REG_OUT_M, 6, (uint8_t*)&dat);
        return dat;
    }

    bool IMUNewData()
    {
        uint8_t status;
        i2c_read(I2C_IMU,REG_STATUS_IMU,1,&status);
        return status&0x03;
    }

    bool MagNewData()
    {
        uint8_t status;
        i2c_read(I2C_MAG,REG_STATUS_M,1,&status);
        return status&0x08;
    }

    bool CheckDevicePresent()
    {
        uint8_t whoami_imu,whoami_mag;
        i2c_read(I2C_IMU, REG_WHO_AM_I, 1, &whoami_imu);
        i2c_read(I2C_MAG, REG_WHO_AM_I, 1, &whoami_mag);
        return (whoami_imu == 0x68) && (whoami_mag == 0x3D);  
    }

    void Init()
    {

        i2c_write(I2C_IMU, REG_CTRL_REG1_G, (uint8_t)0x9B); // Gyro at 238 Hz, 76 Hz bandwidth, 2000 dps range
        i2c_write(I2C_IMU, REG_CTRL_REG6_XL, (uint8_t)0x8B); // Accel at 238 Hz, 105 Hz bandwidth, +/- 16g range

        i2c_write(I2C_MAG, REG_CTRL_REG1_M, (uint8_t)0xFC); // X/Y Axes in ultra high performance 80 Hz mode
        i2c_write(I2C_MAG, REG_CTRL_REG4_M, (uint8_t)0x0C); // Z Axis in ultra high performance
        i2c_write(I2C_MAG, REG_CTRL_REG3_M, (uint8_t)0x00); // Start mag continous conv mode
    }
}
