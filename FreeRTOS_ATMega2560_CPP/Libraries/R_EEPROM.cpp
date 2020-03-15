#include <avr/eeprom.h>
#include <csp/csp_crc32.h>
#include <R_EEPROM.h>


	 unsigned int R_EEPROM::addressspace = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};
	 unsigned int R_EEPROM::blockreserve = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};
	
	int R_EEPROM::qsortcomp(uint16_t in1,uint16_t in2) {
		if (in1<in2) {
			return -1;
			} else if(in1==in2) {
			return 0;
		} else return 1;
		
	};
	
	uint8_t R_EEPROM::init(int add, uint16_t blocksin, void * data) {
		sizeofin=sizeof(*data);
		uint8_t error = 0;
		for (int i=0;i<ADDRESSSPACESIZE;i++) {
			if (addressspace[i]!=0xFFFF) {
				if ((add>addressspace[i])&&(add<addressspace[i]+blockreserve[i])) {
					error = -1;
					~this;
					return error;
				}
				} else {
				addressspace[i]=add;
				blockreserve[i]=(uint16_t(sizeofin)+5)*blocks;
				break;
			}
			if(i==ADDRESSSPACESIZE-1) {
				error=-1;
				~this;
				return error;
			}
		}
		blocks=blocksin;
		startadd=add;
		initr=1;
	};
	
	uint8_t R_EEPROM::read(void * data) {
		
	}


	
	