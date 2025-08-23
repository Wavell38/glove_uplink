#pragma once
#include <cstdint>
#include "hardware/i2c.h"
#include "dev_i2c.hpp"

// ICM-20948 basique (BANK0) : init + lecture acc/gyro
struct Icm20948 {
    uint8_t addr{0x68}; // via TCA
    bool write8(i2c_inst_t* i2c, uint8_t reg, uint8_t v) {
        return i2c_w1(i2c, addr, reg, &v, 1) >= 0;
    }
    bool readN(i2c_inst_t* i2c, uint8_t reg, uint8_t* d, int n) {
        return i2c_r(i2c, addr, reg, d, n) >= 0;
    }
    bool bank(i2c_inst_t* i2c, uint8_t b) { // REG_BANK_SEL 0x7F
        return write8(i2c, 0x7F, (b<<4) & 0x30);
    }
    bool init(i2c_inst_t* i2c) {
        bank(i2c,0);
        // PWR_MGMT_1: CLKSEL=1, sleep=0
        if (!write8(i2c, 0x06, 0x01)) return false;
        // Gyro/Accel enable
        if (!write8(i2c, 0x07, 0x00)) return false; // PWR_MGMT_2
        // Gyro config: ±2000 dps, DLPF on
        bank(i2c,2);
        if (!write8(i2c, 0x01, 0x18)) return false; // GYRO_CONFIG_1: FS=2000dps, DLPF on
        // Accel config: ±2g, DLPF on
        if (!write8(i2c, 0x14, 0x01)) return false; // ACCEL_CONFIG: FS=2g, DLPF on
        // Sample dividers ~200 Hz (simplifié)
        bank(i2c,2);
        if (!write8(i2c, 0x00, 0x04)) return false; // GYRO_SMPLRT_DIV
        if (!write8(i2c, 0x10, 0x04)) return false; // ACCEL_SMPLRT_DIV_1 (high byte=0) -> 200Hz approx
        bank(i2c,0);
        return true;
    }
    bool read_acc_gyro(i2c_inst_t* i2c, int16_t acc[3], int16_t gyr[3], int16_t& temp) {
        // BANK0: ACCEL_XOUT_H 0x2D..0x32 (6), TEMP_OUT 0x39..0x3A (2), GYRO_XOUT_H 0x33..0x38 (6)
        uint8_t b[14];
        if (!readN(i2c, 0x2D, b, 14)) return false;
        auto rd = [&](int i){ return (int16_t)((b[i]<<8)|b[i+1]); };
        acc[0]=rd(0); acc[1]=rd(2); acc[2]=rd(4);
        gyr[0]=rd(8); gyr[1]=rd(10); gyr[2]=rd(12);
        temp = rd(6);
        return true;
    }
    // TODO: mag AK09916 via I2C master interne (à activer plus tard)
};
