
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