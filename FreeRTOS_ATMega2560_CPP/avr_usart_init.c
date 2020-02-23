#include "avr_usart_init.h"
#include <avr/pgmspace.h>
#include <util/delay.h>


/********************************************************************************
                                Main
********************************************************************************/


/********************************************************************************
                                usart Related
********************************************************************************/
void usart_init( uint16_t ubrr) {
	// Set baud rate


	
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;
	UCSR0A |= (1<<U2X0);
	// Enable receiver and transmitter
	UCSR0B = (1<<TXEN0);
	// Set frame format: 8data, 1stop bit
	UCSR0C = (0<<UMSEL00)|(3<<UCSZ00);
}

void usart_putchar(char data) { 
	// Wait for empty transmit buffer
	while ( !(UCSR0A & (1<<UDRE0))) ;
	// Start transmission
	UDR0 = data; 
}

char usart_getchar(void) { 
	// Wait for incoming data
	while ( !(UCSR0A & (1<<RXC0)) );
	// Return the data
	return UDR0;
} 

void usart_pstr_p(const PROGMEM char *s, const int a) {
	  
	  char ch;

	  ch = pgm_read_byte(s++); // read the first character in the string
	  while(ch) // repeat until character is NULL
	  {
		  usart_putchar(ch); // print it
		  ch = pgm_read_byte(s++); // get next character & advance
	  }
	if(a) {
			usart_putchar('\r');
			usart_putchar('\n');
	}
	
}

void usart_pstr(char *s) {
    // loop through entire string
	while (*s) {  
        usart_putchar(*s);
        s++;
    }

}

void usart_pstrl(char *s) {
    // loop through entire string
	usart_pstr(s);

}

unsigned char kbhit(void) {
	//return nonzero if char waiting  polled version
	unsigned char b;
	b=0;
	if(UCSR0A & (1<<RXC0)) b=1;
	return b;
}

void usart_nl(void) {
			usart_putchar('\r');
			usart_putchar('\n');
}

int usart_putchar_printf(char var, FILE *stream) {
	if (var == '\n') usart_putchar('\r');
	usart_putchar(var);
	return 0;
}

