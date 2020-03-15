#define MAXWRITES 100000
#define ADDRESSSPACESIZE 8

class R_EEPROM {
	private:
		uint8_t blockpoint;
		uint8_t sizeofin;
		uint8_t initr;
		uint16_t blocks;
		static uint8_t firstrun;	
		int startadd;
		static unsigned int addressspace[ADDRESSSPACESIZE];
		static unsigned int blockreserve[ADDRESSSPACESIZE];
		int qsortcomp(uint16_t in1,uint16_t in2);
	
	public:
		uint8_t init(int add, uint16_t blocksin, void * data);
		uint8_t write(void * data);
		uint8_t read(void * data);

};

