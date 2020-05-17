#include <stdint.h>

#define REG_ZIN_E 1
#define REG_ZIN_I 49
#define REG_ADD 0
#define REG_RPOS 32
#define REG_APOS 33
#define REG_ERR 34
#define REG_RES_E 3

#define POSREADRETRY 20

class AM4096 {
	
	private:
		volatile uint8_t address;
		uint8_t initcomplete;
	public:
		AM4096(uint8_t addressin);
		uint8_t init(uint8_t invert);
		uint8_t readpos(uint16_t * pos);
		int8_t readabspos(uint16_t * pos);
		uint8_t zeropos(void);
		uint8_t readerror(uint8_t *err);
		uint8_t check(void);
		uint8_t getzero(uint16_t * zerodat);
		uint8_t setzero(uint16_t * data);
		uint8_t initcheck(void);
};