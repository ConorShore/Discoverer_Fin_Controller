#define MAXWRITES 100000

class R_EEPROM {
	private:
	
		uint8_t blockpoint;
		uint8_t sizeoftype;
		determinebpoint(void);
	
	public:
	
		R_EEPROM(uint16_t startadd, uint8_t blocks, void * in);
		uint8_t write(void);
		uint8_t read(void);

}