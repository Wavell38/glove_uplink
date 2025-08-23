#pragma once
#include <cstdint>
#include "hardware/i2c.h"

#include "dev_i2c.hpp"
#include "log.hpp"

int mpu6050_whoami(i2c_inst_t* i2c, uint8_t addr=0x68);
int icm20948_whoami(i2c_inst_t* i2c, uint8_t addr=0x68);
void scan_tca_channels(i2c_inst_t* i2c, uint8_t tca_addr);