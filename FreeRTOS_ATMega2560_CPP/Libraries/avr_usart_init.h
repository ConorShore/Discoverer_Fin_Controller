/********************************************************************************
                                Includes
********************************************************************************/
#include <avr/io.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <avr/pgmspace.h>


#define MYUBRR F_CPU/8/BAUD-1

/********************************************************************************
                                Macros and Defines
********************************************************************************/


/********************************************************************************
                                Function Prototypes
********************************************************************************/
void usart_init(uint16_t ubrr);
char usart_getchar( void ); 
void usart_putchar( char data ); 
void usart_pstr(char *s);
void usart_pstrl(char *s);
void usart_pstr_p(const PROGMEM char *s, const int a);
void usart_nl(void);
unsigned char kbhit(void);
int usart_putchar_printf(char var, FILE *stream);