//functions for triggering enable disable driver and cs on off for SPI


void stepper_1_cs_init(void) {
	PORTJ|=(1<<PJ6); //for enable pin
	PORTH|=(1<<PH6); // for cs pin
	DDRJ&=~(1<<PJ6);
	DDRH|=(1<<PH6);
}

void stepper_1_cson(void) {
	DDRH&=~(1<<PH6);
}

void stepper_1_csoff(void) {
	DDRH|=(1<<PH6);
}

void stepper_1_en(void) {
	DDRJ&=~(1<<PJ6);
}

void stepper_1_dis(void) {
	DDRJ|=(1<<PJ6);
}

void stepper_2_cs_init(void) {
	PORTJ|=(1<<PJ5); //for enable pin
	PORTL|=(1<<PL5); // for cs pin
	DDRJ&=~(1<<PJ5);
	DDRL|=(1<<PL5);
}

void stepper_2_cson(void) {
	DDRL&=~(1<<PL5);
	
}

void stepper_2_csoff(void) {
	DDRL|=(1<<PL5);
}

void stepper_2_en(void) {
	DDRJ&=~(1<<PJ5);
}

void stepper_2_dis(void) {
	DDRJ|=(1<<PJ5);
}