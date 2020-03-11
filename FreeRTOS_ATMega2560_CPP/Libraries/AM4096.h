#include <stdint.h>

#define REG_ZIN_E 1
#define REG_ZIN_I 49
#define REG_ADD 0
#define REG_RPOS 32
#define REG_APOS 33
#define REG_ERR 34


class AM4096 {
	
	private:
	uint8_t address;
	public:
	AM4096(uint8_t addressin);
	uint8_t init(void);
	uint8_t readpos(uint16_t * pos);
	uint8_t readabspos(uint16_t * pos);
	uint8_t zeropos(void);
	uint8_t readerror(uint8_t *err);
	uint8_t check(void);
};