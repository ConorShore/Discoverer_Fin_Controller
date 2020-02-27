#include <stdio.h>
#include <fin.h>
#include <fin_server.h>
#include <SPI/spi.h>
#include <tmc2041.h>
#include <AM4096.h>
#include <tmc2041_glue.h>
#include <avr/wdt.h>
#include <fin_server_backend.h>
#include <csp/csp.h>
#include <csp/arch/csp_thread.h>


uniman_step_config_t uniman_step1_conf = {
	.GCONF=STEPPER_GCONF_DATA(0,0),
	.IHOLD=STEPPER_DEFAULT_IHOLD,
	.IRUN=STEPPER_DEFAULT_IRUN,
	.CHOPCONF=STEPPER_CHOPCONF_DEFAULT
};

uniman_step_config_t uniman_step2_conf = {
	.GCONF=STEPPER_GCONF_DATA(0,0),
	.IHOLD=STEPPER_DEFAULT_IHOLD,
	.IRUN=STEPPER_DEFAULT_IRUN,
	.CHOPCONF=STEPPER_CHOPCONF_DEFAULT
};

uniman_fin_config_t uniman_running_conf = {
	.stepper_config=0x09,
	.stepper_ihold=STEPPER_DEFAULT_IHOLD,
	.stepper_irun=STEPPER_DEFAULT_IRUN,
	.stepper_speed = 60,
	.system_reset_encoder_zero = 0,
	.system_extra = 0
};



csp_thread_handle_t handle_server;
gs_fin_cmd_error_t process_config(uniman_fin_config_t * confin);
void delay_us(uint16_t in);

	
tmc2041 stepper1(&stepper_1_cs_init,&stepper_1_cson,&stepper_1_csoff,&stepper_1_en, &stepper_1_dis,  uniman_step1_conf,STEPPER_1_EEPROM_ADDRESS);
tmc2041 stepper2(&stepper_2_cs_init,&stepper_2_cson,&stepper_2_csoff,&stepper_2_en, &stepper_2_dis,  uniman_step2_conf,STEPPER_2_EEPROM_ADDRESS);

	


	AM4096 encoder1(0x50,4096);
	AM4096 encoder2(0x51,4096);
	AM4096 encoder3(0x52,4096);
	AM4096 encoder4(0x53,4096);
	



// defines for temp sensors
	#define TEMP_MASK1 0x03
	#define TEMP_MASK2 0x02
	#define TEMP_MASK3 0x01
	#define TEMP_MASK4 0x00
	
	#define TEMP_CAL_T0 298.15
	#define TEMP_CAL_T1 276.35
	#define TEMP_CAL_T2 289.55
	#define TEMP_CAL_RT1 27880.0
	#define TEMP_CAL_RT2 15440.0
	//#define TEMP_CAL_BETA (log(TEMP_CAL_RT1/TEMP_CAL_RT2))/((1/TEMP_CAL_T1)-(1/TEMP_CAL_T2))
	#define TEMP_RES_VAL 10000.0
	//#define TEMP_RES_INF TEMP_RES_VAL*(exp((-TEMP_CAL_BETA)/TEMP_CAL_T0))
	
	const double 	temp_cal_beta=16237.12;  //13842.67;
	//const double temp_res_inf = TEMP_RES_VAL*(exp((-temp_cal_beta)/TEMP_CAL_T0));
//

/**
   Processes incoming fin controller command.
 */

gs_fin_cmd_error_t get_fin_status(gs_fin_status_t * status) {

//get setpoints


//get encoder values


//get temps


//get currents

//get mode

}



gs_fin_cmd_error_t set_fin_pos(const gs_fin_positions_t * pos) {}

gs_fin_cmd_error_t set_max_drag(void) {}

gs_fin_cmd_error_t set_min_drag(void) {}
	
gs_fin_cmd_error_t change_ustep(void) {
	
	//When operating at less than 16 times microstepping, be sure to first position to a suitable, symmetric switching position, before changing MRES, otherwise the motor behavior may differ for left and right rotation. For 16 times microstepping, interpolation to 256 microsteps gives best results!
	
}

void setup_temp_sensors(void) {
		ADMUX=0x40;
		ADCSRA=0b10000100; //set adc clock
		ADCSRB=0x00; 
		DIDR0=0x0F; //turn of digital input bufs
		
	}

void read_temp_sensors(uint16_t *array){
		double tempstore=0;
		portENTER_CRITICAL();
		ADMUX=(ADMUX&~(0x0F))|TEMP_MASK1;
		ADCSRA|=(1<<ADSC);
		while(ADCSRA&(1<<ADSC));
		tempstore=1023.0/((double)ADC)-1.0;    
		tempstore= log(tempstore);
		tempstore /= temp_cal_beta;
		tempstore+= 1.0 / (TEMP_CAL_T0);
		*(array)=(uint16_t)(1.0/tempstore);
		
		ADMUX=(ADMUX&~(0x0F))|TEMP_MASK2;
		ADCSRA|=(1<<ADSC);
		while(ADCSRA&(1<<ADSC));
		
		tempstore=1023.0/((double)ADC)-1.0;    
		tempstore= log(tempstore);
		tempstore /= temp_cal_beta;
		tempstore+= 1.0 / (TEMP_CAL_T0);
		*(array+1)=(uint16_t)(1.0/tempstore);
		
		ADMUX=(ADMUX&~(0x0F))|TEMP_MASK3;
		ADCSRA|=(1<<ADSC);
		while(ADCSRA&(1<<ADSC));
		
		tempstore=1023.0/((double)ADC)-1.0;    
		tempstore= log(tempstore);
		tempstore /= temp_cal_beta;
		tempstore+= 1.0 / (TEMP_CAL_T0);
		*(array+2)=(uint16_t)(1.0/tempstore);
		
		ADMUX=(ADMUX&~(0x0F))|TEMP_MASK4;
		ADCSRA|=(1<<ADSC);
		while(ADCSRA&(1<<ADSC));
		
		tempstore=1023.0/((double)ADC)-1.0;    
		tempstore= log(tempstore);
		tempstore /= temp_cal_beta;
		tempstore+= 1.0 / (TEMP_CAL_T0);
		*(array+3)=(uint16_t)(1.0/tempstore);
		portEXIT_CRITICAL();

	
		
	}





CSP_DEFINE_TASK(task_stepper) {

	stepper2.enstep();
	DDRL|=(1<<PL1) | (1<<PL2);
	PORTL|=(1<<PL2);
	uniman_step_reg_32_t tempread = {
			.status=0,
			.address=0x6B,
			.data=0
		};
	
	for(;;) {
	#define deal ((uint16_t) 1024)
	
	if((uniman_running_conf.stepper_config&0x0F)!=0){
		uint16_t stepc=1;
		for (uint16_t i=0; i<(uniman_running_conf.stepper_config&0x0F)-1;i++){
			stepc*=2;
		}
		portENTER_CRITICAL();
		uint16_t i=0;
		while(i<stepc) {
			PORTL^=(1<<PL1);
			i++;
			if(i>=stepc) break;
			delay_us(deal/stepc);
		}
		
		portEXIT_CRITICAL();
	}




		
		vTaskDelay(500/portTICK_PERIOD_MS);

		
	}
	vTaskSuspend(NULL);
}

void delay_us(uint16_t in) {
	while(in--) {
		_delay_us(1);
	}
}


gs_fin_cmd_error_t init_server(void) {
	
	
	gs_fin_cmd_error_t error=FIN_CMD_OK;
	
	
	setup_temp_sensors();
	process_config(&uniman_running_conf);

	stepper1.enstep();
	stepper2.enstep();

	
	if(csp_thread_create(task_server, "SERVER", 270, NULL, 2, &handle_server)) error=FIN_CMD_FAIL;
	
	if(csp_thread_create(task_stepper, "STEP",configMINIMAL_STACK_SIZE+80, NULL, 2, NULL)) error=FIN_CMD_FAIL;

	//should also initalise other things such as temp sensors and steppers here
	
	//also remember to initalise the status frame
	
}

uint8_t step_config_concat(uniman_ustep_mode_t a, uniman_invert_t b) {return a|b;};

gs_fin_cmd_error_t process_config(uniman_fin_config_t * confin) {
	//get invert data
	uniman_running_conf=*confin;
	
	uniman_step1_conf.GCONF=STEPPER_GCONF_DATA(	(uniman_running_conf.stepper_config&(STEPPER_INVA_1))>>4	,(uniman_running_conf.stepper_config&(STEPPER_INVB_1))>>5);
	uniman_step2_conf.GCONF=STEPPER_GCONF_DATA(	(uniman_running_conf.stepper_config&(STEPPER_INVC_1))>>6	,(uniman_running_conf.stepper_config&(STEPPER_INVD_1))>>7);
	
	//get step data
	if((uniman_running_conf.stepper_config&0x0F)!=0) { //if manually setting ustep setting, rahter than auto
		uniman_step1_conf.CHOPCONF=(STEPPER_CHOPCONF_DEFAULT&~(0x0F000000))| (((uint32_t)(9-uniman_running_conf.stepper_config))<<24);
		uniman_step2_conf.CHOPCONF=(STEPPER_CHOPCONF_DEFAULT&~(0x0F000000))| (((uint32_t)(9-uniman_running_conf.stepper_config))<<24);
		
	}
	
	// do the currents, these translate directly
	uniman_step1_conf.IRUN=uniman_running_conf.stepper_irun;
	uniman_step2_conf.IRUN=uniman_running_conf.stepper_irun;
	
	uniman_step1_conf.IHOLD=uniman_running_conf.stepper_ihold;
	uniman_step2_conf.IHOLD=uniman_running_conf.stepper_ihold;
	
	stepper1.updateconfig(&uniman_step1_conf);
	stepper2.updateconfig(&uniman_step2_conf);

}


