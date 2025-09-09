#pragma once
#include <cstdint>
#include "hardware/i2c.h"
#include "log.hpp"
#include "pico/error.h"  // en haut du fichier

inline bool tca_select(i2c_inst_t* i2c, uint8_t tca_addr, uint8_t ch) {
    uint8_t data = (ch <= 7) ? (1u << ch) : 0x00;
    int wr = i2c_write_timeout_us(i2c, tca_addr, &data, 1, false, 3000);
    if (wr != 1) {
        const char* why = (wr == PICO_ERROR_TIMEOUT) ? "TIMEOUT" :
                          (wr == PICO_ERROR_GENERIC) ? "NACK" : "ERR";
        LOGE("TCA sel ch%d: I2C WR %s (addr=0x%02X, ret=%d)", ch, why, tca_addr, wr);
        return false;
    }
    sleep_us(100);
    return true;
}

inline int i2c_w1(i2c_inst_t* i2c, uint8_t addr, uint8_t reg, const uint8_t* data, int len) {
    uint8_t buf[1+32]; if (len>32) return -1;
    buf[0]=reg; for (int i=0;i<len;i++) buf[1+i]=data[i];
    return i2c_write_timeout_us(i2c, addr, buf, 1+len, false, /*timeout_us=*/5000);
}

inline int i2c_r(i2c_inst_t* i2c, uint8_t addr, uint8_t reg, uint8_t* data, int len) {
    int w = i2c_write_timeout_us(i2c, addr, &reg, 1, true, /*timeout_us=*/5000);
    if (w != 1) return -1;
    return i2c_read_timeout_us(i2c, addr, data, len, false, /*timeout_us=*/5000);
}