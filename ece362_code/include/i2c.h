
// #ifndef __I2C_H
#define __I2C_H
#include "stdlib.h"

void enable_ports();
void init_i2c();
void i2c_start(uint32_t targadr, uint8_t size, uint8_t dir);
void i2c_stop();
void i2c_waitidle();
void i2c_clearnack();
int i2c_checknack();
void accel_write(uint16_t loc, uint8_t *data, uint8_t len);
void accel_read(uint16_t loc, uint8_t data[], uint8_t len);
