#include <stdio.h>

void I2C_init(void);
uint8_t I2C_write(const uint8_t address,const uint8_t * data, const uint8_t number,const uint8_t stop);
uint8_t I2C_read(const uint8_t address,uint8_t reg, uint8_t * data, const uint8_t number);