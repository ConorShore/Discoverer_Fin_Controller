#include <stdio.h>

void I2C_init(void);
uint8_t I2C_write(uint8_t address,uint8_t * data, uint8_t number);
uint8_t I2C_read(uint8_t address,uint8_t reg, uint8_t * data, uint8_t number);
