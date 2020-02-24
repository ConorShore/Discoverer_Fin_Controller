//functions for triggering enable disable driver and cs on off for SPI

#include <stdio.h>


void stepper_1_cs_init(void) {
	DDRJ|=(1<<PJ6); //for enable pin
	DDRH|=(1<<PH6); // for cs pin
	PORTJ&=~(1<<PJ6);
	PORTH|=(1<<PH6);
}

void stepper_1_cson(void) {
	PORTH&=~(1<<PH6);
}

void stepper_1_csoff(void) {
	PORTH|=(1<<PH6);
}

void stepper_1_en(void) {
	PORTJ&=~(1<<PJ6);
}

void stepper_1_dis(void) {
	PORTJ|=(1<<PJ6);
}

void stepper_2_cs_init(void) {
	DDRJ|=(1<<PJ5); //for enable pin
	DDRL|=(1<<PL5); // for cs pin
	PORTJ&=~(1<<PJ5);
	PORTL|=(1<<PL5);
}

void stepper_2_cson(void) {
	PORTL&=~(1<<PL5);
	
}

void stepper_2_csoff(void) {
	PORTL|=(1<<PL5);
}

void stepper_2_en(void) {
	PORTJ&=~(1<<PJ5);
}

void stepper_2_dis(void) {
	PORTJ|=(1<<PJ5);
}