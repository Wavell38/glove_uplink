#pragma once
#include <cstdint>
#include "hardware/i2c.h"
#include "dev_i2c.hpp"

struct Mpu6050 {
    uint8_t addr{0x68}; // via TCA, tous peuvent être 0x68
    bool init(i2c_inst_t* i2c) {
        // PWR_MGMT_1: clock PLL, wake
        uint8_t v = 0x01; if (i2c_w1(i2c, addr, 0x6B, &v, 1) < 0) return false;
        // GYRO_CONFIG: FS_SEL=3 (±2000 dps)
        v = 0x18; if (i2c_w1(i2c, addr, 0x1B, &v, 1) < 0) return false;
        // ACCEL_CONFIG: AFS_SEL=0 (±2g)
        v = 0x00; if (i2c_w1(i2c, addr, 0x1C, &v, 1) < 0) return false;
        // CONFIG (DLPF): 2 -> ~94 Hz
        v = 0x02; if (i2c_w1(i2c, addr, 0x1A, &v, 1) < 0) return false;
        // SMPLRT_DIV: 4 -> 200 Hz (1kHz/(1+4))
        v = 0x04; if (i2c_w1(i2c, addr, 0x19, &v, 1) < 0) return false;
        return true;
    }
    // 14 bytes: ACC[6], TEMP[2], GYR[6]
    bool read14(i2c_inst_t* i2c, int16_t acc[3], int16_t& temp, int16_t gyr[3]) {
        uint8_t buf[14];
        if (i2c_r(i2c, addr, 0x3B, buf, 14) < 0) return false;
        auto rd = [&](int idx){ return (int16_t)((buf[idx]<<8) | buf[idx+1]); };
        acc[0]=rd(0); acc[1]=rd(2); acc[2]=rd(4);
        temp = rd(6);
        gyr[0]=rd(8); gyr[1]=rd(10); gyr[2]=rd(12);
        return true;
    }
};
