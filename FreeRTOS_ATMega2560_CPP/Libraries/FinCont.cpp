#include "FinCont.h"
#include <avr/io.h>
#include <avr/interrupt.h>

int leden=0;

void enableleds(void) {
	DDRD|= (1<<PD5) | (1<<PD6) | (1<<PD7);
	leden=1;
}

void contled(int con, int led) {
	if (leden==0) {return;}
	char tarled = 0;
	switch (led)
	{
	case 0 : tarled = PD5;
		break;
	case 1 : tarled = PD6;
		break;
	case 2 : tarled = PD7;
		break;
	}
		switch (con)
		{
	case 0 : PORTD ^= (1<<tarled);
		break;
	case 1 : PORTD |= (1<<tarled);
		break;
	case 2 : PORTD &= ~(1<<tarled);
		break;
	}
	
}

void pwrredinit(void) {
	PRR0|=0b01100000; //shuts down TIM0 and 2
	PRR1|=0b00111111; //shuts down TIM3-5 and USART1-3
	
}

void timeoutstart(int tim) {
	// remember power save
	cli();
	PRR1&=~(1<<PRTIM3);
	
	TCCR3A = 0;
	TCCR3B = 0;
	TCNT3 = 0;
	
	OCR3A=tim;
	
	TCCR3B|=(1<<WGM32);	
	TCCR3B |= (1 << CS32);
	TIFR3|=(1<<OCF3A);
	sei();
	
}

void timeoutreset(void) {
		TCNT3 = 0;
}

int timeoutcheck(void) {
	
	if((TIFR3&2)>>OCF3A) {
		cli();
			TCCR3A = 0;
			TCCR3B = 0;
			TCNT3 = 0;
			TIFR3&=~(1<<OCF3A);
			PRR1|=(1<<PRTIM3);
			sei();
		return 1;
	} else {
		return 0;
		}
return 0;
}

void timeoutstart2_us(int tim) {
	// remember power save
	cli();

	
	TCCR4A = 0;
	TCCR4B = 0;
	TCNT4 = 0;
	
	OCR4A=(tim*2);
	
	TCCR4B|=(1<<WGM42);	
	TCCR4B |= (1 << CS41); //x16 scaling
	TIFR4|=(1<<OCF4A);
	sei();
	
}

void timeoutreset_us(void) {
		TCNT4 = 0;
}

int timeoutcheck2_us(void) {
	
	if((TIFR4&2)>>OCF4A) {
		cli();
			TCCR4A = 0;
			TCCR4B = 0;
			TCNT4 = 0;
			TIFR4&=~(1<<OCF4A);

			sei();
		return 1;
	} else {
		return 0;
		}
return 0;
}

