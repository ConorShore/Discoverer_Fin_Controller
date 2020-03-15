 #include <stdio.h>
#include <avr/eeprom.h>
#include <csp/csp_crc32.h>
#include <R_EEPROM.h>

	unsigned int R_EEPROM::addressspace[ADDRESSSPACESIZE];
	unsigned int R_EEPROM::blockreserve[ADDRESSSPACESIZE];
	 uint8_t R_EEPROM::initc;
	 
	 uint8_t R_EEPROM::initcheck(void) {
		 if(initc==0||initr==0) {
			return 0; 
		 } else {
			return 1; 
		 }
	 }
	
	 void R_EEPROM::initclass(void) {
		if(initc==0) {
			for(int i=0;i<ADDRESSSPACESIZE;i++) {
				addressspace[i] = 0xFFFF;
				blockreserve[i] = 0xFFFF;
	 		}
			 initc=1;
		}
	};

	
	int R_EEPROM::qsortcomp(uint16_t in1,uint16_t in2) {
		if (in1<in2) {
			return -1;
			} else if(in1==in2) {
			return 0;
		} else return 1;
		
	};
	
	uint8_t R_EEPROM::init(uint16_t add, uint16_t blocksin, uint16_t sizeofdata) {
		if(initcheck()) return -1;
		sizeofin=sizeofdata;
		uint8_t error = 0;
		uint16_t resamount=(uint16_t(sizeofin)+CRCSIZE+COUNTERSIZE)*blocksin;
		printf("add %d resam %u sizeofin %d\n",add,resamount,sizeofin);
		for (int i=0;i<ADDRESSSPACESIZE;i++) {
					printf("%x\n",addressspace[i]);
					
			if (addressspace[i]!=0xFFFF) { //array initalised to 0xFFFF, so if not this there is data here
				if (((add>=addressspace[i])&&(add<addressspace[i]+blockreserve[i]))||
						((add+resamount>addressspace[i])&&(add+resamount<=addressspace[i]+blockreserve[i]))) { 
							//does the address collide with preexisting data
					error = -1;
					return error;
				}
			} else {
				addressspace[i]=add;
				blockreserve[i]=resamount; 
				blocks=blocksin;
				break;
			}
			if(i==ADDRESSSPACESIZE-1) {
				error=-1;
			
				return error;
			}
		}
		blocks=blocksin;
		startadd=add;
		initr=1;
		
		return error;
	};
	
	uint8_t R_EEPROM::read(uint8_t * data) {
		
	}
	R_EEPROM::R_EEPROM(void) {
		initr=0;
	}
	
	uint8_t R_EEPROM::write(uint8_t * data) {

		
		//eeprom_read_block((uint8_t *) startadd);
		
	}


	
	