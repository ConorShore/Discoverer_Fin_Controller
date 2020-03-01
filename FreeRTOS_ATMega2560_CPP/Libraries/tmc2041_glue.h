//functions for triggering enable disable driver and cs on off for SPI

#include <stdio.h>


void stepper_1_pin_init(void) {
	DDRJ|=(1<<PJ6); //for enable pin
	DDRH|=(1<<PH6) | (1<<PH2) | (1<<PH3) | (1<<PH4) | (1<<PH5); // for cs pin
	PORTJ&=~(1<<PJ6);
	PORTH|=(1<<PH6) | (1<<PH3) | (1<<PH5);
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

void stepper_1_tstep(uint8_t a) {
	if (a) {
		PORTH^=(1<<PH4);
		} else {
		PORTH^=(1<<PH2);
	}
}

void stepper_1_dir(uint8_t a, uint8_t t) { //a is which stepper 0 is 1 1 is 2, t is what to do, 2 is toggle, 1 is on 0 is off
	if(a) {
		if (t==2) {
			PORTH^=(1<<PH5);
			} else {
			if (t==1) {
				PORTH|=(1<<PH5);
				} else {
				PORTH&=~(1<<PH5);
			}
		}
		} else {
		if (t==2) {
			PORTH^=(1<<PH3);
			} else {
			if (t==1) {
				PORTH|=(1<<PH3);
				} else {
				PORTH&=~(1<<PH3);
			}
		}
	}
}

void stepper_2_pin_init(void) {
	DDRJ|=(1<<PJ5); //for enable pin
	DDRL|=(1<<PL5) | (1<<PL1) | (1<<PL2) | (1<<PL3)| (1<<PL4) ; // for cs pin
	PORTJ&=~(1<<PJ5);
	PORTL|=(1<<PL5) | (1<<PL2) | (1<<PL4);
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

void stepper_2_tstep(uint8_t a) {
	if (a) {
		PORTL^=(1<<PL3);
	} else {
		PORTL^=(1<<PL1);
	}
}

void stepper_2_dir(uint8_t a, uint8_t t) { //a is which stepper 0 is 1 1 is 2, t is what to do, 2 is toggle, 1 is on 0 is off
	if(a) {
		if (t==2) {
			PORTL^=(1<<PL4);
		} else {
			if (t==1) {
				PORTL|=(1<<PL4);
			} else {
				PORTL&=~(1<<PL4);
			}
		}
	} else {
		if (t==2) {
			PORTL^=(1<<PL2);
		} else {
			if (t==1) {
				PORTL|=(1<<PL2);
			} else {
				PORTL&=~(1<<PL2);
			}
		}
	}
}