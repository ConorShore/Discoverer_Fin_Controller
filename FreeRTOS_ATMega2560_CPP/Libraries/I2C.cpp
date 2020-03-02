#include <avr/interrupt.h>
#include <avr/io.h>
#include <FreeRTOS/portmacro.h>
#include <util/delay.h>
#include <stdio.h>

// some defs for error etc from hardware
#define START 0x08
#define RSTART	0x10
#define MT_SLA_ACK 0x18
#define MT_DATA_ACK 0x28
#define MR_SLA_ACK 0x40
#define MR_DATA_ACK 0x50

void I2C_init(void) {
	portENTER_CRITICAL();
	TWBR=12; //12 =400kHz
	TWSR=0;
	TWCR|= (1<<TWEA) | (1<<TWEN) | (1<<TWINT);
	PORTD|=((1<<PD0) | (1<<PD1)); 
	portEXIT_CRITICAL();
}



uint8_t I2C_write_ns(uint8_t address,uint8_t * data, uint8_t number) { 
	//write command without stop at the end
	// the write command is split up so the read command can use this version
	// at the start of a read frame
	 
	portENTER_CRITICAL();
	TWCR|=(1<<TWSTA);
	
	while (!(TWCR & (1<<TWINT))); //wait for I2C to be ready
	if ((TWSR & 0xF8) != START) {
		TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
		portEXIT_CRITICAL();
		return -1;
	}
	
	TWDR = (address<<1);
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	while (!(TWCR & (1<<TWINT)));
	
	if ((TWSR & 0xF8) !=MT_SLA_ACK) {
		TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
		portEXIT_CRITICAL();
		return -1;
	}
	if(data!=NULL) {
		for(int i=0;i<number;i++) {
			TWDR = *(data+i);
			TWCR = (1<<TWINT) | (1<<TWEN);
			while (!(TWCR & (1<<TWINT)));
	
			if ((TWSR & 0xF8) != MT_DATA_ACK) {
				TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
				portEXIT_CRITICAL();
				return -1;
			}
		}
	}
	portEXIT_CRITICAL();
	return 0;
}

uint8_t I2C_write(uint8_t address,uint8_t * data, uint8_t number) {
	if(I2C_write_ns(address,data,number)==0) {
		TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
		return 0;
		} else {
		return -1;
	}
	
}

uint8_t I2C_read(uint8_t address,uint8_t reg,uint8_t * data, uint8_t number) {
	
	if(I2C_write_ns(address,&reg,0)!=0){
		TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
		portEXIT_CRITICAL();
		return 2;
		} //write address and reg request
	
	portENTER_CRITICAL();
	TWCR|=(1<<TWSTA); //repeated start
	
	while (!(TWCR & (1<<TWINT))); //wait for I2C to be ready
	if ((TWSR & 0xF8) != RSTART) { //check RS was transmitted
		TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
		portEXIT_CRITICAL();
		return 3;
	}
	
	TWDR = (address<<1) | 0x01; //read add on the end
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	while (!(TWCR & (1<<TWINT)));
	
	if ((TWSR & 0xF8) !=MR_SLA_ACK) { //wait for ack of address
		TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
		portEXIT_CRITICAL();
		return 4;
	}
	
	for(int i=0;i<number;i++) {
		if ((TWSR & 0xF8) != MR_DATA_ACK) { //if data
			TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
			portEXIT_CRITICAL();
			return 5;
		} else {
		
		*(data+i)=TWDR;

		}
	}

	TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
	portEXIT_CRITICAL();
	return 0;
	
	
	
}

