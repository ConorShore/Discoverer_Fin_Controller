#include <SPI/spi.h>
#include <fin.h>
#include <tmc2041.h>
#include <stdint.h>


	void tmc2041::startSPI(void) {
		SPI.beginTransaction(SPISettings(STEPPER_SPI_CLK, MSBFIRST, SPI_MODE3));
		cson();
	};
	void tmc2041::endSPI(void) {
		csoff();
		SPI.endTransaction();
		
	}
	
	void tmc2041::readreg(uniman_step_reg_t * databack) {
		tmc2041::startSPI();
		SPI.transfer((databack->address&0x7F)|0x80);
		databack->status=SPI.transfer(0);
		SPI.transfer(&databack->data,4);
		tmc2041::endSPI();
		
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
	
