#pragma once
#include <cstdint>
#include "hardware/i2c.h"

// TCA9548A (0x70..0x77)
/*inline bool tca_select(i2c_inst_t* i2c, uint8_t tca_addr, uint8_t channel) {
    uint8_t v = (1u << channel);
    return i2c_write_blocking(i2c, tca_addr, &v, 1, false) == 1;
}
inline int i2c_w1(i2c_inst_t* i2c, uint8_t addr, uint8_t reg, const uint8_t* data, int len) {
    uint8_t buf[1+32]; if (len>32) return -1;
    buf[0]=reg; for(int i=0;i<len;i++) buf[1+i]=data[i];
    return i2c_write_blocking(i2c, addr, buf, 1+len, false);
}
inline int i2c_r(i2c_inst_t* i2c, uint8_t addr, uint8_t reg, uint8_t* data, int len) {
    if (i2c_write_blocking(i2c, addr, &reg, 1, true) != 1) return -1;
    return i2c_read_blocking(i2c, addr, data, len, false);
}*/

inline bool tca_select(i2c_inst_t* i2c, uint8_t tca_addr, uint8_t channel) {
    uint8_t v = (1u << channel);
    int r = i2c_write_timeout_us(i2c, tca_addr, &v, 1, false, 200); // 1 ms
    return r == 1;
}

inline int i2c_w1(i2c_inst_t* i2c, uint8_t addr, uint8_t reg, const uint8_t* data, int len) {
    uint8_t buf[1+32]; if (len>32) return -1;
    buf[0]=reg; for(int i=0;i<len;i++) buf[1+i]=data[i];
    return i2c_write_timeout_us(i2c, addr, buf, 1+len, false, 300);
}

inline int i2c_r(i2c_inst_t* i2c, uint8_t addr, uint8_t reg, uint8_t* data, int len) {
    int w = i2c_write_timeout_us(i2c, addr, &reg, 1, true, 200);
    if (w != 1) return -1;
    return i2c_read_timeout_us(i2c, addr, data, len, false, 500);
}
