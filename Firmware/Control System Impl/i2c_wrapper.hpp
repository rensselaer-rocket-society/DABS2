#include <Wire.h>

inline void i2c_read(uint8_t addr, uint8_t reg, uint32_t n, uint8_t* buffer)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start
    Wire.requestFrom(addr, n); // Request the data...
    for(uint32_t i = 0; i < n; ++i)
    {
        *buffer = Wire.read();
        ++buffer;
    }
}

inline void i2c_write(uint8_t addr, uint8_t reg, uint8_t value)
{  
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}