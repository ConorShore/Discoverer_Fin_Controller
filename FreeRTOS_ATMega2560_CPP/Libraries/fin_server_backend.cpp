﻿#include <stdio.h>
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
#include <csp/arch/csp_queue.h>
#include <avr/eeprom.h>
#include <FinCont.h>


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

gs_fin_config_t uniman_running_conf = {
	.stepper_config=0x09,
	.stepper_ihold=STEPPER_DEFAULT_IHOLD,
	.stepper_irun=STEPPER_DEFAULT_IRUN,
	.stepper_speed = 60,
	.system_reset_encoder_zero = (1<<5),
	.system_extra = 0
};

#define EEPROM_RUN_CONF_ADD 0x00

csp_queue_handle_t uniman_stepper_q;

#define STEPPER_QUEUE_LENGTH 5

typedef struct stepper_cmd {
	uint8_t direction;
	uint16_t tarsteps;
	uint16_t cursteps;
}stepper_cmd_t;


csp_thread_handle_t handle_server;
gs_fin_cmd_error_t process_config(gs_fin_config_t * confin);
void delay_us(uint16_t in);

	
tmc2041 stepper1(&stepper_1_pin_init,&stepper_1_cson,&stepper_1_csoff,&stepper_1_en, 
&stepper_1_dis, &stepper_1_tstep, &stepper_1_dir, 
uniman_step1_conf,STEPPER_1_EEPROM_ADDRESS);

tmc2041 stepper2(&stepper_2_pin_init,&stepper_2_cson,&stepper_2_csoff,&stepper_2_en, 
&stepper_2_dis,  &stepper_2_tstep, &stepper_2_dir, 
uniman_step2_conf,STEPPER_2_EEPROM_ADDRESS);

	


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



gs_fin_cmd_error_t set_fin_pos(const gs_fin_positions_t * pos) {
	
	//TODO - add eeprom save
	
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

	uint16_t recbuf=0;
	stepper_cmd_t stepcmd[4];
	bool inmove[4]= {0,0,0,0};
		
	for(;;) {
		TickType_t funcstarttime=xTaskGetTickCount();
		
		while(csp_queue_dequeue(uniman_stepper_q,&recbuf,0)==1) {
			stepcmd[ (recbuf&0xC000)>>14 ] = {	//select correct stepper command
					.direction = (uint8_t) ((recbuf&0x2000)>>13), //get direction from packet
					.tarsteps = recbuf&0x1FFF // get steps
					}; 
					inmove[(recbuf&0xC000)>>14]=1;
					if (inmove[0]==1||inmove[1]==1) stepper1.enstep();
					if (inmove[2]==1||inmove[3]==1) stepper2.enstep();
				csp_log_info("Queue rec for stepper %d	dir:%d	steps:%d",((recbuf&0xC000)>>14),stepcmd[ (recbuf&0xC000)>>14 ].direction,stepcmd[ (recbuf&0xC000)>>14 ].tarsteps);
		}
		
		
		uint16_t stepc=7;
		if(inmove[0]) {
			stepper1.dirfunc(0,stepcmd[0].direction&0x01);
		}
		if(inmove[1]) {
			stepper1.dirfunc(1,stepcmd[1].direction&0x01);
		}
		if(inmove[2]) {
			stepper2.dirfunc(0,stepcmd[2].direction&0x01);
		}
		if(inmove[3]) {
			stepper2.dirfunc(1,stepcmd[3].direction&0x01);
		}
		
		for (uint16_t i=0; i<(uniman_running_conf.stepper_config&0x0F)-1;i++){
			stepc*=2;
		}
		//portENTER_CRITICAL();
		uint16_t i=0;
		while(i<stepc) {
			if(inmove[0]) {
				stepper1.stepfunc(0);
			}
			if(inmove[1]) {
				stepper1.stepfunc(1);
			}
			if(inmove[2]) {
				stepper2.stepfunc(0);
			}
			if(inmove[3]) {
				stepper2.stepfunc(1);
			}
			i++;
			timeoutstart2_us(STEPPER_FSTEP_DELAY>>((uniman_running_conf.stepper_config&0x0F)-1));
			while(!timeoutcheck2_us());
			//delay_us(STEPPER_FSTEP_DELAY>>((uniman_running_conf.stepper_config&0x0F)-1));
		}
		//_delay_ms(1);
				if(inmove[0]) {
					stepper1.dirfunc(0,(!(stepcmd[0].direction))&0x01);
				}
				if(inmove[1]) {
					stepper1.dirfunc(1,(!(stepcmd[1].direction))&0x01);
				}
				if(inmove[2]) {
					stepper2.dirfunc(0,(!(stepcmd[2].direction))&0x01);
				}
				if(inmove[3]) {
					stepper2.dirfunc(1,(!(stepcmd[3].direction))&0x01);
				}
		stepc=5;
		for (uint16_t i=0; i<(uniman_running_conf.stepper_config&0x0F)-1;i++){
			stepc*=2;
		}
		i=0;
		while(i<stepc) {
			if(inmove[0]) {
				stepper1.stepfunc(0);
			}
			if(inmove[1]) {
				stepper1.stepfunc(1);
			}
			if(inmove[2]) {
				stepper2.stepfunc(0);
			}
			if(inmove[3]) {
				stepper2.stepfunc(1);
			}
			i++;
			if(i>=stepc) break;
			
			//implement feedback somewhere
			
			timeoutstart2_us(STEPPER_FSTEP_DELAY>>((uniman_running_conf.stepper_config&0x0F)-1));
			while(!timeoutcheck2_us());
			//delay_us(STEPPER_FSTEP_DELAY>>((uniman_running_conf.stepper_config&0x0F)-1));
		}
		
			for (int i=0;i<4;i++) {
				if(inmove[i]) stepcmd[i].cursteps++;
			}
			
			for(int i=0;i<4;i++) {
				if((inmove[i]==1)&&(stepcmd[i].cursteps==stepcmd[i].tarsteps)) inmove[i]=0;
			}
			

		
		//vTaskDelay((uint16_t)(1000*(uint32_t)60)/(uniman_running_conf.stepper_speed*(uint32_t)portTICK_PERIOD_MS));
		vTaskDelayUntil(&funcstarttime,(uint16_t)(1000*(uint32_t)60)/(uniman_running_conf.stepper_speed*(uint32_t)portTICK_PERIOD_MS));
		if ((uniman_running_conf.system_reset_encoder_zero&(1<<5))&&inmove[0]==0&&inmove[1]==0) stepper1.disstep();
		if ((uniman_running_conf.system_reset_encoder_zero&(1<<5))&&inmove[2]==0&&inmove[3]==0) stepper2.disstep();
	}
	
	vTaskSuspend(NULL);
}




gs_fin_cmd_error_t init_server(void) {
	
	
	gs_fin_cmd_error_t error=FIN_CMD_OK;
	
	
	setup_temp_sensors();


	
	if(csp_thread_create(task_server, "SERVER", 270, NULL, 2, &handle_server)) error=FIN_CMD_FAIL;
	
	if(csp_thread_create(task_stepper, "STEP",configMINIMAL_STACK_SIZE+80, NULL, 1, NULL)) error=FIN_CMD_FAIL;
	
		uniman_stepper_q = csp_queue_create(STEPPER_QUEUE_LENGTH,sizeof(uint16_t));
		if (uniman_stepper_q==NULL) error =FIN_CMD_FAIL;
		
		
		// stepper q layout
		// [15:14] Which stepper, 00 is stepper 1, 01 is stepper 2 etc.
		// [13] direction
		// [12:0] no of steps
		
		
		uint16_t p=0x0005;
		
		csp_queue_enqueue(uniman_stepper_q,&p,1000);
			p=0x4005;
		
		csp_queue_enqueue(uniman_stepper_q,&p,1000);
			p=0x8005;
		
		csp_queue_enqueue(uniman_stepper_q,&p,1000);
			p=0xC005;
		
		csp_queue_enqueue(uniman_stepper_q,&p,1000);
		


		if(load_fin_config()) error=FIN_CMD_FAIL;

		


	
	return error;
	
	
	//TODO - also remember to initalise the status frame
	
}

uint8_t step_config_concat(uniman_ustep_mode_t a, uniman_invert_t b) {return a|b;};

gs_fin_cmd_error_t process_config(gs_fin_config_t * confin) {
	//get invert data
	
	uniman_running_conf=*confin;
	
	uniman_step1_conf.GCONF=STEPPER_GCONF_DATA(	(uniman_running_conf.stepper_config&(STEPPER_INVA_1))>>4	,(uniman_running_conf.stepper_config&(STEPPER_INVB_1))>>5);
	uniman_step2_conf.GCONF=STEPPER_GCONF_DATA(	(uniman_running_conf.stepper_config&(STEPPER_INVC_1))>>6	,(uniman_running_conf.stepper_config&(STEPPER_INVD_1))>>7);
	
	//get step data
	if((uniman_running_conf.stepper_config&0x0F)!=0) { //if manually setting ustep setting, rahter than auto
// 		uniman_step1_conf.CHOPCONF=(STEPPER_CHOPCONF_DEFAULT&~(0x0F000000))| (((uint32_t)(9-uniman_running_conf.stepper_config))<<24);
// 		uniman_step2_conf.CHOPCONF=(STEPPER_CHOPCONF_DEFAULT&~(0x0F000000))| (((uint32_t)(9-uniman_running_conf.stepper_config))<<24);
// 		
		uniman_step1_conf.CHOPCONF=STEPPER_CHOPCONF_DEFAULT;
 		uniman_step2_conf.CHOPCONF=STEPPER_CHOPCONF_DEFAULT;
	}
	

	
	// do the currents, these translate directly
	uniman_step1_conf.IRUN=uniman_running_conf.stepper_irun;
	uniman_step2_conf.IRUN=uniman_running_conf.stepper_irun;
	
	uniman_step1_conf.IHOLD=uniman_running_conf.stepper_ihold;
	uniman_step2_conf.IHOLD=uniman_running_conf.stepper_ihold;
	
	stepper1.updateconfig(&uniman_step1_conf,confin);
	stepper2.updateconfig(&uniman_step2_conf,confin);
	
	return FIN_CMD_OK;

}

gs_fin_cmd_error_t get_fin_config(gs_fin_config_t * conf) {
	*conf=uniman_running_conf;
	csp_log_info("Returning running config");
	return FIN_CMD_OK;
}

gs_fin_cmd_error_t set_fin_config(const gs_fin_config_t * conf) {
	uniman_running_conf=*conf;
	process_config(&uniman_running_conf);
	csp_log_info("Setting running config");
	//TODO - action things like reset, zeroing
	
	
	return FIN_CMD_OK;
}

gs_fin_cmd_error_t load_fin_config(void) {
	gs_fin_config_t temp;
	gs_fin_cmd_error_t error = FIN_CMD_OK;
	csp_log_info("Loading config from EEPROM");
	portENTER_CRITICAL();

	eeprom_read_block(&temp,(uint16_t*) EEPROM_RUN_CONF_ADD,sizeof(gs_fin_config_t));
	uniman_running_conf=temp;
	portEXIT_CRITICAL();
	csp_log_info("Error = %d",error);
	return FIN_CMD_OK;
}


gs_fin_cmd_error_t save_fin_config(void) {
	gs_fin_config_t temp;
	gs_fin_cmd_error_t error = FIN_CMD_OK;
	csp_log_info("Saving running config to EEPROM");
	portENTER_CRITICAL();
	
eeprom_write_block(&uniman_running_conf,(uint16_t*) EEPROM_RUN_CONF_ADD,sizeof(gs_fin_config_t));

eeprom_read_block(&temp,(uint16_t*) EEPROM_RUN_CONF_ADD,sizeof(gs_fin_config_t));

if(temp.stepper_config!=uniman_running_conf.stepper_config) error = FIN_CMD_FAIL;
if(temp.stepper_ihold!=uniman_running_conf.stepper_ihold) error = FIN_CMD_FAIL;
if(temp.stepper_irun!=uniman_running_conf.stepper_irun) error = FIN_CMD_FAIL;
if(temp.stepper_speed!=uniman_running_conf.stepper_speed) error = FIN_CMD_FAIL;
if(temp.system_reset_encoder_zero!=uniman_running_conf.system_reset_encoder_zero) error = FIN_CMD_FAIL;
if(temp.system_extra!=uniman_running_conf.system_extra) error = FIN_CMD_FAIL;
portEXIT_CRITICAL();
csp_log_info("Error = %d",error);

return error;
	


}

void print_conf(gs_fin_config_t * confin) {
			printf("%x\n",confin->stepper_config);
		printf("%x\n",confin->stepper_ihold);
		printf("%x\n",confin->stepper_irun);
		printf("%x\n",confin->stepper_speed);
		printf("%x\n",confin->system_reset_encoder_zero);
		printf("%x\n\n",confin->system_extra);
}


