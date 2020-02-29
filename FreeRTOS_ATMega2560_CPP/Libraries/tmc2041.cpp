#include <SPI/spi.h>
#include <fin.h>
#include <tmc2041.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <FreeRTOS/portmacro.h>
#include <avr/delay.h>

void delay_us(uint16_t in) {
	while(in--) {
		_delay_us(1);
	}
}

	void tmc2041::startSPI(void) {
			cson();
		SPI.beginTransaction(SPISettings(STEPPER_SPI_CLK, MSBFIRST, SPI_MODE3));
	
	};
	void tmc2041::endSPI(void) {
		csoff();
		SPI.endTransaction();
		
	}
	
	void tmc2041::readreg(uniman_step_reg_t * databack) {
		startSPI();
		SPI.transfer((databack->address)&0x7F);
		for (int i =0; i<4;i++) SPI.transfer(0);
		endSPI();
		startSPI();
		//printf("\n\n%x\n",((databack->address)&0x7F));
		databack->status=SPI.transfer(0);
		SPI.transfer(&databack->data,4);
		endSPI();
	}
	
	void tmc2041::readreg(uniman_step_reg_32_t * databack) {
		startSPI();
		SPI.transfer((databack->address)&0x7F);
		for (int i =0; i<4;i++) SPI.transfer(0);
		endSPI();
		startSPI();
		//printf("\n\n%x\n",((databack->address)&0x7F));
		databack->status=SPI.transfer(0);
		for (int i =0; i<4;i++) {
			databack->data = databack->data  <<	8;
			databack->data += SPI.transfer(0);
			
		}
		endSPI();
	}
	



	
	void tmc2041::writereg(uniman_step_reg_t * databack,uint8_t amount) {
		startSPI();
		SPI.transfer(((databack->address)&0x7F)|0x80);
		for (int i =0; i<amount;i++) SPI.transfer(databack->data[i]);
		endSPI();
	}
	
	void tmc2041::writereg(uniman_step_reg_32_t * databack) {

		startSPI();
		SPI.transfer(((databack->address)&0x7F)|0x80);
		for (int i =0; i<4;i++) {
			uint8_t temp = ((databack->data)>>(8*(3-i)))&0xFF;
			SPI.transfer(temp);
		}
		endSPI();
		
	}
	

	
	

	tmc2041::tmc2041(void (csinitin()),void (csonin()),
	void (csoffin()),void (enstepin()), void (disstepin()), void(stepfuncin(uint8_t)),void(dirfuncin(uint8_t,uint8_t)),
	uniman_step_config_t configin,uint8_t eeprom_addressin) : init_pins(csinitin), cson(csonin), csoff(csoffin) , enstep(enstepin),
	disstep(disstepin), stepfunc(stepfuncin), dirfunc(dirfuncin) {
		
		config=configin;
		eeprom_address=eeprom_addressin;
		init_pins();
		SPI.begin();
		
		
	};
	
	gs_fin_cmd_error_t tmc2041::updateconfig(uniman_step_config_t * confin,uniman_fin_config_t * confin2) {
		runconf=*confin2;
		config=*confin;
		
		gs_fin_cmd_error_t error=FIN_CMD_OK;
		
		uniman_step_reg_32_t tempwrite = {
			.status=0,
			.address=STEPPER_GCONF_ADD,
			.data=(uint32_t)config.GCONF
		};
		
		uniman_step_reg_32_t tempread = {
			.status=0,
			.address=tempwrite.address,
			.data=0
		};
		
		writereg(&tempwrite);
		readreg(&tempread);

		if (tempwrite.data!=tempread.data) {
		error=FIN_CMD_FAIL;

		}
		
		
		tempwrite = {
			.status=0,
			.address=STEPPER_ISET1_ADD,
			.data=	((((uint32_t)config.IRUN)<<8)|config.IHOLD)  | (((uint32_t)STEPPER_DEFAULT_IHOLDDELAY)<<16)
		};
		
		tempread.address=tempwrite.address;
		tempread.data=0;
		
		writereg(&tempwrite);
			
		tempwrite = {
			.status=0,
			.address=STEPPER_ISET2_ADD,
			.data=	((((uint32_t)config.IRUN)<<8)|config.IHOLD) | (((uint32_t)STEPPER_DEFAULT_IHOLDDELAY)<<16)
		};
		
		tempread.address=tempwrite.address;
		tempread.data=0;
		
		writereg(&tempwrite);
			
		tempwrite = {
			.status=0,
			.address=STEPPER_CHOPCONF1_ADD,
			.data=	config.CHOPCONF
		};
		
		tempread.address=tempwrite.address;
		tempread.data=0;
		
		writereg(&tempwrite);
		readreg(&tempread);
	
		if (tempwrite.data!=tempread.data) {
		error=FIN_CMD_FAIL;

		}
		tempwrite = {
			.status=0,
			.address=STEPPER_CHOPCONF2_ADD,
			.data=	config.CHOPCONF
		};
		
		tempread.address=tempwrite.address;
		tempread.data=0;
		
		writereg(&tempwrite);
		readreg(&tempread);
		
		if (tempwrite.data!=tempread.data) {
		error=FIN_CMD_FAIL;

		}
		
	return error;
			
	}

	void tmc2041::step(uint8_t a, uint8_t dir) { //a is stepper 0=1, 1=2, 0 is reverse, 1 is forward
		

	
	if((runconf.stepper_config&0x0F)!=0){
		uint16_t stepc=4;
		dirfunc(a,dir+1);
		for (uint16_t i=0; i<(runconf.stepper_config&0x0F)-1;i++){
			stepc*=2;
		}
		portENTER_CRITICAL();
		uint16_t i=0;
		while(i<stepc) {
			stepfunc(a);
			i++;
			delay_us(STEPPER_FSTEP_DELAY>>((runconf.stepper_config&0x0F)-1));
		}
		//_delay_ms(1);
		dirfunc(a,dir+2);
		stepc=2;
		for (uint16_t i=0; i<(runconf.stepper_config&0x0F)-1;i++){
			stepc*=2;
		}
		i=0;
		while(i<stepc) {
			stepfunc(a);
			i++;
			if(i>=stepc) break;
			delay_us(STEPPER_FSTEP_DELAY>>((runconf.stepper_config&0x0F)-1));
		}
		
		portEXIT_CRITICAL();
		
	}
	}

	
