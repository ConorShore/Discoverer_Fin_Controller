#include <avr/interrupt.h>
#include <avr/io.h>
#include <FreeRTOS/portmacro.h>
#include <avr/delay.h>
#define START 0x08
#define MT_SLA_ACK 0x18

void I2C_init(void) {
	portENTER_CRITICAL();
	TWBR=50; //12
	TWSR=0;
	TWCR|= (1<<TWEA) | (1<<TWEN) | (1<<TWINT);
	PORTD|=((1<<PD0) | (1<<PD1)); 
	portEXIT_CRITICAL();
}

uint8_t I2C_write(uint8_t address,uint8_t * data, uint8_t number) {
	TWCR|=(1<<TWSTA);
	
	_delay_us(1000);
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != START) {
		TWCR = (1<<TWINT)|(1<<TWEN)|
(1<<TWSTO);
		return 1;
	}
	
	TWDR = (address<<1);
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	while (!(TWCR & (1<<TWINT)));
	
	if ((TWSR & 0xF8) !=MT_SLA_ACK) {
		TWCR = (1<<TWINT)|(1<<TWEN)|
(1<<TWSTO);
		return 2;
	}

	TWCR = (1<<TWINT)|(1<<TWEN)|
(1<<TWSTO);
	return 0;
}

uint8_t I2C_read(uint8_t address,uint8_t * data, uint8_t number) {
	
}

