#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <stdint.h>

namespace LSM{

    const float ACCEL_TO_MPSPS = 0.0071784678;
    const float MAG_TO_UT = 14e-3;
    const float GYRO_TO_RADPS = 0.00122173048;

    struct vec_i16_t
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    struct IMUData {
        vec_i16_t accel;
        vec_i16_t gyro;
    };

    IMUData ReadIMU();

    vec_i16_t ReadMag();

    bool IMUNewData();
    bool MagNewData();

    bool CheckDevicePresent();

    void Init();
}

#endif