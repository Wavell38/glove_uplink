#include "scan.hpp"
#include "dev_i2c.hpp"
#include "log.hpp"

// WHO_AM_I helpers
int mpu6050_whoami(i2c_inst_t* i2c, uint8_t addr) {
    uint8_t v=0; if (i2c_r(i2c, addr, 0x75, &v, 1) < 0) return -1; return v;
}
int icm20948_whoami(i2c_inst_t* i2c, uint8_t addr) {
    uint8_t v=0; if (i2c_r(i2c, addr, 0x00, &v, 1) < 0) return -1; return v;
}

// Cherche le TCA @0x70..0x77 (retourne -1 si absent)
int find_tca_addr(i2c_inst_t* i2c) {
    for (uint8_t a=0x70; a<=0x77; ++a) {
        // "ping" I2C : écriture 0 octet
        int r = i2c_write_timeout_us(i2c, a, nullptr, 0, false, 3000);
        if (r >= 0) { LOGI("TCA candidate found @0x%02X", a); return a; }
    }
    LOGE("No TCA found on 0x70..0x77");
    return -1;
}

// Scan par canal : ping des adresses + WHO_AM_I ciblés
void scan_tca_channels(i2c_inst_t* i2c, uint8_t tca_addr) {
    LOGI("Scanning TCA9548A at 0x%02X ...", tca_addr);
    for (int ch=0; ch<8; ++ch) {
        if (!tca_select(i2c, tca_addr, ch)) { LOGE(" ch%d: select FAILED", ch); continue; }
        sleep_us(200);

        // Ping large (0x03..0x77)
        int found = 0;
        char line[256]; int ofs = 0;
        ofs += snprintf(line+ofs, sizeof(line)-ofs, " ch%d devices:", ch);
        for (uint8_t a=0x03; a<=0x77; ++a) {
            if (a == tca_addr) continue;
            int r = i2c_write_timeout_us(i2c, a, nullptr, 0, false, 2000);
            if (r >= 0) { ofs += snprintf(line+ofs, sizeof(line)-ofs, " 0x%02X", a); found++; }
        }
        LOGI("%s (n=%d)", line, found);

        // WHO_AM_I ciblés aux adresses usuelles
        int wm68 = mpu6050_whoami(i2c, 0x68);
        int wm69 = mpu6050_whoami(i2c, 0x69);
        int wi68 = icm20948_whoami(i2c, 0x68);

        if (wm68 == 0x68)      LOGI(" ch%d: MPU6050 @0x68 (WHO=0x68)", ch);
        if (wm69 == 0x68)      LOGI(" ch%d: MPU6050 @0x69 (WHO=0x68)", ch);
        if (wi68 == 0xEA)      LOGI(" ch%d: ICM-20948 @0x68 (WHO=0xEA)", ch);
        if (wm68 != 0x68 && wm69 != 0x68 && wi68 != 0xEA && found==0)
            LOGW(" ch%d: no known device (WHO mpu@68=%d mpu@69=%d icm@68=%d)", ch, wm68, wm69, wi68);
    }
}
