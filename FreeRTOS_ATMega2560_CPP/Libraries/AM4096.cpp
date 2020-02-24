#include <AM4096.h>
#include <stdint.h>


	AM4096::AM4096(uint8_t addressin, uint16_t resin) {
		address=addressin;
		resolution=resin;
	}
