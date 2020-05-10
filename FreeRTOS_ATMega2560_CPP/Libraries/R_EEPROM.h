#define MAXWRITES 65530
#define ADDRESSSPACESIZE 8
#define BUFFSIZE 64

#define CRCTYPE uint32_t


typedef CRCTYPE crc_t;


#define CRCSIZE sizeof(CRCTYPE)


class R_EEPROM {
	private:
		uint16_t blockpoint;
		uint8_t sizeofin;
		uint8_t initr;
		static uint8_t initc;
		uint16_t blocks;
		static uint8_t firstrun;	
		int startadd;
		int curaddress;
		static unsigned int addressspace[ADDRESSSPACESIZE];
		static unsigned int blockreserve[ADDRESSSPACESIZE];
		int qsortcomp(uint16_t in1,uint16_t in2);
		uint8_t initcheck(void);
		uint8_t incrementblock(void);
		uint8_t internalwrite(const void * data);
	
	public:
		R_EEPROM(void);
		uint8_t begin(uint16_t add, uint16_t blocksin, uint16_t sizeofdata, void * data);
		uint8_t write(const void * data);
		uint8_t read(void * data);

};

