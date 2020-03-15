#include <avr/eeprom.h>
#include <csp/csp_crc32.h>

	R_EEPROM::determinebpoint(void) {
		
	}

	R_EEPROM::R_EEPROM(uint16_t startadd, uint8_t blocks, void * in) {
		sizeoftype=sizeof(*in);
		
	}