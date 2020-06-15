#ifndef MPL3115A2_H
#define MPL3115A2_H

#include <stdint.h>

namespace MPL{

    const float ALT_TO_M = 1.0/16.0;
    const float TEMP_TO_C = 1.0/16.0;

    struct AltTempData {
        int32_t alt; //In 16ths of a meter
        int16_t temp; //In 16ths of a degree Celsius
    };

    AltTempData ReadData();

    bool CheckAndRead(AltTempData* result);

    AltTempData BlockingRead();

    bool IsDataReady();

    void SetStandby(bool stby);

    void RequestData();

    bool CheckDevicePresent();

    void Init();
}

#endif