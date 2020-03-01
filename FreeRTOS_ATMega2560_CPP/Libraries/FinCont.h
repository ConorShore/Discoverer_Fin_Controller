
#include <avr/io.h>

#define TOGCON 0
#define ONCON 1
#define OFFCON 2


void pwrredinit(void); // some power saving commands
void timeoutstart(int tim);
int timeoutcheck(void);
void timeoutreset(void);
void enableleds(void);
void contled(int con, int led);
void timeoutstart2_us(int tim);
void timeoutreset_us(void);
int timeoutcheck2_us(void);