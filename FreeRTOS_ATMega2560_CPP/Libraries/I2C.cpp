#include <avr/interrupt.h>
#include <avr/io.h>
#include <FreeRTOS/portmacro.h>
#include <avr/delay.h>
#define START 0x08
#define MT_SLA_ACK 0x18

void I2C_init(void) {
	portENTER_CRITICAL();
	TWBR=10;
	TWSR&=~((1<<TWPS0)|(1<<TWPS1));
	TWCR|= (1<<TWEA) | (1<<TWEN) | (1<<TWINT);
	portEXIT_CRITICAL();
}

uint8_t I2C_write(uint8_t address,uint8_t * data, uint8_t number) {
	TWCR|=(1<<TWSTA);
	
	_delay_us(1000);
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != START) return 1;
	
	TWDR = (address<<1) | 0x01;
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	while (!(TWCR & (1<<TWINT)));
	
	if ((TWSR & 0xF8) !=MT_SLA_ACK) return 2;
	
	return 0;
}

uint8_t I2C_read(uint8_t address,uint8_t * data, uint8_t number) {
	
}

