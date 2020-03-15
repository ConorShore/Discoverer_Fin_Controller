#define MAXWRITES 100000
#define ADDRESSSPACESIZE 8
#define CRCTYPE uint32_t
#define COUNTERTYPE uint16_t

typedef CRCTYPE crc_t;
typedef COUNTERTYPE count_t;

#define CRCSIZE sizeof(CRCTYPE)
#define COUNTERSIZE sizeof(COUNTERTYPE)

class R_EEPROM {
	private:
		uint8_t blockpoint;
		uint8_t sizeofin;
		uint8_t initr;
		static uint8_t initc;
		uint16_t blocks;
		static uint8_t firstrun;	
		int startadd;
		static unsigned int addressspace[ADDRESSSPACESIZE];
		static unsigned int blockreserve[ADDRESSSPACESIZE];
		int qsortcomp(uint16_t in1,uint16_t in2);
		uint8_t initcheck(void);
	
	public:
		R_EEPROM(void);
		void initclass(void);
		uint8_t init(uint16_t add, uint16_t blocksin, uint16_t sizeofdata);
		uint8_t write(uint8_t * data);
		uint8_t read(uint8_t * data);

};

