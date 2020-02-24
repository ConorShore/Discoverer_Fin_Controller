#include <SPI/spi.h>
#include <fin.h>
#include <tmc2041.h>
#include <stdint.h>
#include <stdio.h>




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
	

	
	

	tmc2041::tmc2041(void (csinitin()),void (csonin()),void (csoffin()),void (enstepin()), void (disstepin()),
	uniman_step_config_t configin) : init_cs_en(csinitin), cson(csonin), csoff(csoffin) , enstep(enstepin),
	disstep(disstepin) {
		
		config=configin;
		init_cs_en();
		SPI.begin();
		
		
	};
	
	gs_fin_cmd_error_t tmc2041::set_speed(uint8_t) {};
	gs_fin_cmd_error_t tmc2041::set_pos1(uint16_t pos, uint8_t speed) {};
	gs_fin_cmd_error_t tmc2041::set_pos2(uint16_t pos, uint8_t speed) {};
	
