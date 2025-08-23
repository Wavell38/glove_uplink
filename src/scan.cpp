#include "scan.hpp"

// Renvoie 0x68 pour MPU6050, -1 si erreur
int mpu6050_whoami(i2c_inst_t* i2c, uint8_t addr) {
    uint8_t v=0; if (i2c_r(i2c, addr, 0x75, &v, 1) < 0) return -1; return v;
}
// Renvoie 0xEA pour ICM-20948 (BANK0), -1 si erreur
int icm20948_whoami(i2c_inst_t* i2c, uint8_t addr) {
    uint8_t v=0; if (i2c_r(i2c, addr, 0x00, &v, 1) < 0) return -1; return v;
}

void scan_tca_channels(i2c_inst_t* i2c, uint8_t tca_addr) {
    LOGI("Scanning TCA9548A at 0x%02X ...", tca_addr);
    for (int ch=0; ch<8; ++ch) {
        if (!tca_select(i2c, tca_addr, ch)) { LOGE(" TCA ch%d: select FAILED", ch); continue; }
        sleep_us(5); // souvent 0 suffirait
        int w_mpu = mpu6050_whoami(i2c);
        int w_icm = icm20948_whoami(i2c);
        if (w_mpu == 0x68)      LOGI(" ch%d: MPU6050 detected (WHO_AM_I=0x68)", ch);
        else if (w_icm == 0xEA) LOGI(" ch%d: ICM-20948 detected (WHO_AM_I=0xEA)", ch);
        else                    LOGW(" ch%d: no known device (mpu=%d, icm=%d)", ch, w_mpu, w_icm);
    }
}