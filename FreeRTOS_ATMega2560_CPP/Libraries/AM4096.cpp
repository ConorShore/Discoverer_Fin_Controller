#include <AM4096.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <I2C.h>
#include <util/delay.h>
#include <csp/csp.h>

//assumbed 4096 res


AM4096::AM4096(uint8_t addressin) {
	address=addressin;
	initcomplete=0;
}

uint8_t AM4096::init(uint8_t invert) {
 	uint8_t tempdata[3] = {REG_ADD,0b00011101,address}; //first byte is mem address
// 	if(I2C_write(address,tempdata,3,1)!=0) return -1;
// 	_delay_ms(30);
	if(I2C_read(address,REG_ZIN_E,&tempdata[1],2)!=0) return -1; //get current zin setting
	if (invert==1) {
		tempdata[1] |= (1<<4);
	} else {
		tempdata[1] &=~(1<<4);
	}
	tempdata[0]=REG_ZIN_E;
	if(I2C_write(address,tempdata,3,1)!=0) return -1; //this inverts direction
	_delay_ms(30);
	tempdata[2] = 0b00000000;
	tempdata[1] = 0b00001000;
	tempdata[0]=REG_RES_E;
	if(I2C_write(address,tempdata,3,1)!=0) return -1; //this set resolution
	_delay_ms(30);
	
	
	initcomplete=1;
	return 0;

}

uint8_t AM4096::initcheck(void) {
// 		if(initcomplete==0) {
// 			if(init(address)!=0) return -1;
// 			else return 0;
// 		}
		return 0;
}

uint8_t AM4096::check(void) {
	if(I2C_write(address,NULL,1,1)!=0) {
		return -1;
		} else {
		return 0;
	}
}

uint8_t AM4096::readpos(uint16_t * pos) {
	uint8_t tempdata[2] = {0,0};
		if(initcheck()!=0) return -1;
	
	uint8_t complete=-1;

	for(int i=0;i<POSREADRETRY;i++) {
		if(I2C_read(address,REG_RPOS,tempdata,2)==0) {
			complete=1;
			break;
		}
	}

	if (complete!=1) {
		return -1;
	} else {
	
		if((tempdata[0]&0xF0)==0) { //if this bit is set, data is invalid
			*pos = (((uint16_t)tempdata[0])<<8)|tempdata[1];
			return 0;
			} else {
			return -1;
		}
	}
}

int8_t AM4096::readabspos(uint16_t * pos) {
	uint8_t tempdata[2] = {0,0};
		if(initcheck()!=0) return -1;
		
	uint8_t complete=-1;
	printf("s\n");
	for(int i=0;i<POSREADRETRY;i++) {
		if(I2C_read(address,REG_APOS,tempdata,2)==0) {
			complete=1;
			break;
		}
	
	}
	printf("e\n");
	if (complete!=1) {
		return -1;
	} else {
	
		if((tempdata[0]&0xF0)==0) { //if this bit is set, data is invalid
			*pos = (((uint16_t)tempdata[0])<<8)|tempdata[1];
			return 0;
			} else {
			return -1;
		}
	}
}

uint8_t AM4096::zeropos(void) {
	uint8_t tempdata[3] = {0,0,0};
	uint16_t curpos=0;
	if(initcheck()!=0) return -1;
	
	if(I2C_read(address,REG_ZIN_I,tempdata,2)!=0) return -1; //get current zin setting
	uint8_t zinhigh=tempdata[0]&0xF000; //save top nibble
	
	if(readabspos(&curpos)!=0) return -1; //get current abs pos
	
	tempdata[0]=REG_ZIN_E;
	tempdata[1]=zinhigh	|((curpos&0x0F00)>>8);
	tempdata[2]=(uint8_t)(curpos&0x00FF);
	if(I2C_write(address,tempdata,3,1)!=0) return -1; //save new zin to encoder EEPROM
	
	return 0;
	
	
}

uint8_t AM4096::readerror(uint8_t * err) {
	if(initcheck()!=0) return -1;
	
	uint8_t tempdata[2] = {0,0};
	
	if(I2C_read(address,REG_ERR,tempdata,2)!=0) return -1;
	
	*err=tempdata[0]&0xC0;
	
	return 0;
}

uint8_t AM4096::getzero(uint16_t * zerodat) {
	uint8_t tempdata[2] = {0,0};
		if(initcheck()!=0) return -1;
	if(I2C_read(address,REG_ZIN_I,tempdata,2)!=0) return -1; //get current zin setting
	*zerodat=((uint16_t(tempdata[1]&0x0F))<<8)+tempdata[0];
	return 0;
	
}

uint8_t AM4096::setzero(uint16_t * data) {
		
		uint8_t tempdata[3] = {REG_ZIN_E,0,0}; //first byte is mem address
			if(initcheck()!=0) return -1;
		if(I2C_read(address,REG_ZIN_I,&tempdata[1],2)!=0) return -1; //get current zin setting
		tempdata[1]|=uint8_t(((*data)&0x0F00)>>8);
		tempdata[2]=uint8_t((*data)&0x00FF);
		if(I2C_write(address,tempdata,3,1)!=0) return -1;
		_delay_ms(30);
		return 0;
}
