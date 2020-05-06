#include <stdio.h>
#include <stdlib.h>
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
#include <I2C.h>
#include <R_EEPROM.h>

R_EEPROM running_conf_EEPROM;
R_EEPROM last_pos_rec;


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

uniman_step_config_t uniman_fstep_conf = {
	.GCONF=STEPPER_GCONF_DATA(0,0),
	.IHOLD=STEPPER_DEFAULT_IHOLD,
	.IRUN=STEPPER_DEFAULT_IRUN,
	.CHOPCONF=0x080300C3	
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
#define EEPROM_LAST_POS_REC 0x20

csp_queue_handle_t uniman_stepper_q;

#define STEPPER_QUEUE_LENGTH 8

typedef struct stepper_cmd {
	uint8_t direction;
	uint16_t tarsteps;
	uint16_t cursteps;
	uint16_t startenc;
	uint16_t tarenc;
	uint8_t retry;
	uint16_t trackenc;
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

void print_conf(gs_fin_config_t * confin);
void read_temp_sensors(uint16_t *array);


gs_fin_status_t uniman_status = {
	.pos_set_points= {0,0,0,0},
	.encoder_pos = {0,0,0,0},
	.temperatures = {0,0,0,0},
	.currents = {0,0,0,0},
	.mode = GS_FIN_MODE_INIT,
	.status_code=0
};

uint16_t step_set_points[4] = {0,0,0,0};

uint16_t enc_tar_points[4] = {0,0,0,0};

	


	AM4096 encoder1(0x50);
	AM4096 encoder2(0x51);
 	AM4096 encoder3(0x52);
 	AM4096 encoder4(0x53);
	
	



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
	
	//TODO - remove this test code
// 	gs_fin_positions_t test2 = {
// 		.pos_fin_a=0x01F1,
// 		.pos_fin_b=0x02F2,
// 		.pos_fin_c=0x03F3,
// 		.pos_fin_d=0x04F4
// 	};
// 	
// 	gs_fin_status_t test = {
// 		.pos_set_points=test2,
// 		.encoder_pos=test2,
// 		.temperatures={0x05F5,0x06F6,0x07F7,0x08F8},
// 		.currents={0x09F9,0x10F0,0x11F1,0x12F2},
// 		.mode=GS_FIN_MODE_CUSTOM,
// 		.status_code=7
// 	};
// 	
// 	*status=test;
// 	return FIN_CMD_OK;

//get setpoints, probably will done during normal operation

	status->pos_set_points=uniman_status.pos_set_points;
	
	gs_fin_cmd_error_t error=FIN_CMD_OK;


//get encoder values
	float tempd=0;
	uint16_t temp16=0;
	uint8_t temp8=0;
	
	encoder1.readerror(&temp8);
	printf("%x ",temp8);

	if(encoder1.readpos(&temp16)!=0) error=FIN_CMD_FAIL;
	tempd=(float)temp16;
	tempd/=1.13777777778;

	status->encoder_pos.pos_fin_a=uint16_t(temp16);
	
	temp16=0;
	if(encoder2.readpos(&temp16)!=0) error=FIN_CMD_FAIL;
	tempd=(float)temp16;
	tempd/=1.13777777778;

	status->encoder_pos.pos_fin_b=uint16_t(temp16);
	
		encoder2.readerror(&temp8);
	printf("%x ",temp8);
	
	temp16=0;
	if(encoder3.readpos(&temp16)!=0) error=FIN_CMD_FAIL;
	tempd=(float)temp16;
	tempd/=1.13777777778;

	status->encoder_pos.pos_fin_c=uint16_t(temp16);
	
		encoder3.readerror(&temp8);
	printf("%x ",temp8);
	
	temp16=0;
	if(encoder4.readpos(&temp16)!=0) error=FIN_CMD_FAIL;
	tempd=(float)temp16;
	tempd/=1.13777777778;

	status->encoder_pos.pos_fin_d=uint16_t(temp16);

		encoder4.readerror(&temp8);
	printf("%x \n",temp8);

//get temps
	read_temp_sensors(&status->temperatures[0]);
	//printf("temps %d %d %d %d\n",status->temperatures[0],status->temperatures[1],status->temperatures[2],status->temperatures[3]);


	status->currents[0] = 0;
	status->currents[1] = 0;
	status->currents[2] = 0;
	status->currents[3] = 0;
	

//get mode
	status->mode=uniman_status.mode;
	
	
	//TODO - remove following test statement
	error=FIN_CMD_OK;
	return error;

}

gs_fin_cmd_error_t set_fin_pos_ns(const gs_fin_positions_t * pos) {
	
	gs_fin_cmd_error_t error=FIN_CMD_OK;
	
	

	
	uint16_t temp16=0;

	float tempd[3]={0.0,0.0,0.0};
	float reqpos=0;
	
	//TODO - catch input errors
	for (int i=0; i<4; i++) {
		

		
		uint8_t internalerror=0;
		tempd[0]=0;
		tempd[1]=0;
		tempd[2]=0;
		temp16=0;
		reqpos=0;
		switch (i) {
			case 0:
		
				if(encoder1.readpos(&temp16)!=0) {
					error=FIN_CMD_FAIL;
					internalerror=1;
				}
				reqpos=float(pos->pos_fin_a);
			break;
			
			case 1:
				if(encoder2.readpos(&temp16)!=0) {
					error=FIN_CMD_FAIL;
					internalerror=1;
				}
				reqpos=float(pos->pos_fin_b);
			break;
		
			case 2:
				if(encoder3.readpos(&temp16)!=0) {
					error=FIN_CMD_FAIL;
					internalerror=1;
				}
				reqpos=float(pos->pos_fin_c);
			break;

			case 3:
				if(encoder4.readpos(&temp16)!=0) {
					error=FIN_CMD_FAIL;
					internalerror=1;
				}
				reqpos=float(pos->pos_fin_d);	
			break;		
		}
		
		//printf("Step %d, enc %d, to %d \n\n",i,temp16,uint16_t(reqpos));
		
		if (reqpos==60001) {
			uint16_t commandinc=((i)<<14) | (0<<13) | (0&0x1FFF);
			if(csp_queue_enqueue(uniman_stepper_q,&commandinc,1000)!=0) error=FIN_CMD_FAIL;
			printf("pasezn\n");
			continue;
		} else if(reqpos>3600) {
			csp_log_error("invalid data for stepper from comms");
			continue;
		} else {
			switch (i) {
				case 0:
				uniman_status.pos_set_points.pos_fin_a=pos->pos_fin_a;
				break;
				case 1:
				uniman_status.pos_set_points.pos_fin_b=pos->pos_fin_b;
				break;
				case 2:
				uniman_status.pos_set_points.pos_fin_c=pos->pos_fin_c;
				break;
				case 3:
				uniman_status.pos_set_points.pos_fin_d=pos->pos_fin_d;
				break;
			}
		}
		
		if(internalerror!=0)  {
			csp_log_error("Enc %d error",i+1);
			continue;
		}
		

		
		
		tempd[0]=(float)temp16;
		tempd[0]/=1.13777777778;
		
	
		tempd[1]=(reqpos-tempd[0])-3600;
		while(abs(tempd[1])>=3600) {
			if(tempd[1]<0) {
				tempd[1]+=3600;
			} else {
				tempd[1]-=3600;
			}
		}
		
		
		tempd[2]=(3600-tempd[0]+reqpos);
		
		while(abs(tempd[2])>=3600) {
			if(tempd[2]<0) {
				tempd[2]+=3600;
			} else {
				tempd[2]-=3600;
			}
		}
		
		csp_log_info("req pos %d dir 0 %d dir 1 %d",int16_t(reqpos),int16_t(tempd[1]),int16_t(tempd[2]));
		uint8_t direction=0;
		
	
	
		if(abs(tempd[1])<abs(tempd[2])) {
			temp16=uint16_t(abs(tempd[1]/8.333333333));
			if(tempd[1]<0) {
				direction=1;
			}
		} else {
			temp16=uint16_t(abs(tempd[2]/8.333333333));
			if(tempd[2]<0) {
				direction=1;
			}
		}
		
			
			uint16_t commandinc=((i)<<14) | (direction<<13) | (temp16&0x1FFF);
		
			if(csp_queue_enqueue(uniman_stepper_q,&commandinc,1000)!=0) error=FIN_CMD_FAIL;

	}
	
	//TODO - remove this test statement
	error=FIN_CMD_OK;
	return error;
}

gs_fin_cmd_error_t set_fin_pos(const gs_fin_positions_t * pos) {
	
	last_pos_rec.write(pos);
	
	return set_fin_pos_ns(pos);

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
	#define BASEOVERSTEPS 2;
	uint8_t oversteps = BASEOVERSTEPS;
	#define RETRYMAX 2
	#define RETRYMARGIN 16
	uint16_t recbuf=0;
	uint8_t intervalcount=0;
	#define INTERVALPERIOD 5
	#define INTERVALMARGINFACTOR 0.5
	
	#define MAXOVERSTEPS 2
	const uint8_t intervalstep=uint8_t(float((INTERVALPERIOD)*0.833333333)*11.3777777778);
	const uint16_t intervalmargin = uint16_t(INTERVALMARGINFACTOR*float(intervalstep));
	stepper_cmd_t stepcmd[4];
	stepcmd[0]={0,0,0,0,0,0,0};
		stepcmd[1]=stepcmd[0];
		stepcmd[2]=stepcmd[0];
		stepcmd[3]=stepcmd[0];
	bool inmove[4]= {0,0,0,0};
		stepper1.enstep();
 		stepper2.enstep();
	for(;;) {
		TickType_t funcstarttime=xTaskGetTickCount();
		
		while(csp_queue_dequeue(uniman_stepper_q,&recbuf,0)==1) {
			if((recbuf&0x1FFF)==0) {
				continue;
			}
			stepcmd[ (recbuf&0xC000)>>14 ].direction = (uint8_t) ((recbuf&0x2000)>>13);
			stepcmd[ (recbuf&0xC000)>>14 ].tarsteps=recbuf&0x1FFF;
			stepcmd[ (recbuf&0xC000)>>14 ].cursteps=0;
		
		
				inmove[(recbuf&0xC000)>>14]=1;
				if((inmove[0]+inmove[1]+inmove[2]+inmove[3])!=0) uniman_status.mode=GS_FIN_MODE_MOVING;
				
				uint16_t posrec=0;
				switch ((recbuf&0xC000)>>14) {
					case 0:
						encoder1.readpos(&posrec);
					break;
					
					case 1:
						encoder2.readpos(&posrec);
					break;	
					
					case 2:
						encoder3.readpos(&posrec);
					break;
					
					case 3:
						encoder4.readpos(&posrec);
					break;
				}
			
				stepcmd[ (recbuf&0xC000)>>14 ].startenc=posrec;
				stepcmd[ (recbuf&0xC000)>>14 ].trackenc=posrec;
				if(stepcmd[ (recbuf&0xC000)>>14 ].direction==0) {
					stepcmd[ (recbuf&0xC000)>>14 ].tarenc=posrec+(uint16_t)(((float)stepcmd[ (recbuf&0xC000)>>14 ].tarsteps)*9.48148148148);
				} else {
					int16_t temptarenc=posrec-(uint16_t)(((float)stepcmd[ (recbuf&0xC000)>>14 ].tarsteps)*9.48148148148);
					if(temptarenc>=0) {
					stepcmd[ (recbuf&0xC000)>>14 ].tarenc=temptarenc;
					} else {
						stepcmd[ (recbuf&0xC000)>>14 ].tarenc=4096+temptarenc;
					}
				}
				if(stepcmd[ (recbuf&0xC000)>>14 ].tarenc>4095) stepcmd[ (recbuf&0xC000)>>14 ].tarenc=stepcmd[ (recbuf&0xC000)>>14 ].tarenc%4096;
				
				csp_log_info("Queue rec for stepper %d	dir:%u	steps:%u cur e:%u tar e %u\n",((recbuf&0xC000)>>14)+1,stepcmd[ (recbuf&0xC000)>>14 ].direction,stepcmd[ (recbuf&0xC000)>>14 ].tarsteps,stepcmd[ (recbuf&0xC000)>>14 ].startenc,stepcmd[ (recbuf&0xC000)>>14 ].tarenc);
		}
		
		
		uint16_t stepc=oversteps;

		
		if(intervalcount>=INTERVALPERIOD) {
			intervalcount=0;
			uint8_t errorsteps=0;
			for(int i=0;i<4;i++) {
				uint16_t tpos=0;
				if(inmove[i]==1) {
					switch (i) {
						case 0:
							encoder1.readpos(&tpos);
						break;
					
						case 1:
							encoder2.readpos(&tpos);
						break;	
					
						case 2:
							encoder3.readpos(&tpos);
						break;
					
						case 3:
							encoder4.readpos(&tpos);
						break;
					}
					int16_t error=abs(tpos-stepcmd[i].trackenc);
					uint16_t error1=0;
					uint16_t error2=0;
					int16_t track=tpos;
					
					
					
					int16_t test1=4096+tpos-stepcmd[i].trackenc;
					int16_t test2=4096-tpos+stepcmd[i].trackenc;				
					while(abs(test1)>=4096) {
						if(test1<0) {
							test1+=4096;
						} else {
							test1-=4096;
						}
					}
		
					while(abs(test2)>=4096) {
						if(test2<0) {
							test2+=4096;
						} else {
							test2-=4096;
						}
					}
					
					if(abs(test1)<abs(test2)) {
						error=test1;
					} else {
						error=test2;
					}
					
					
					//csp_log_info("delta a %u",error);

					error-=intervalstep;
					error=abs(error);
					
					//csp_log_info("tpos %u track %u cur %d ex %u mar %u",
						//tpos,stepcmd[i].trackenc,error,intervalstep,intervalmargin);
	
					if(error>intervalmargin) {
						//csp_log_error("Missed steps on %u",i+1);
						errorsteps++;
						
						//csp_log_info("new over %d",oversteps);
					} 
					if (errorsteps==0) {

						//csp_log_info("overstep to normal");
					}
					stepcmd[i].trackenc=tpos;
					
				}
		
			}
			if (errorsteps>0) {
				oversteps*=2;
				if(oversteps>MAXOVERSTEPS) {
					oversteps=MAXOVERSTEPS;
				}
				} else {
				oversteps=BASEOVERSTEPS;
			}
		}
		

		intervalcount++;
				
		if (inmove[0]==1||inmove[1]==1) stepper1.enstep();
		if (inmove[2]==1||inmove[3]==1) stepper2.enstep();
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
		
		for (uint8_t i=0; i<(uniman_running_conf.stepper_config&0x0F)-1;i++){
			stepc*=2;
		}
		//portENTER_CRITICAL();
		uint16_t i=0;

		//printf("\n");
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
		stepc=oversteps-2;
		for (uint8_t i=0; i<(uniman_running_conf.stepper_config&0x0F)-1;i++){
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

			timeoutstart2_us(STEPPER_FSTEP_DELAY>>((uniman_running_conf.stepper_config&0x0F)-1));
			while(!timeoutcheck2_us());
			//delay_us(STEPPER_FSTEP_DELAY>>((uniman_running_conf.stepper_config&0x0F)-1));
		}
		
			for (int i=0;i<4;i++) {
				if(inmove[i]) stepcmd[i].cursteps++;
			}
			
			
			

		vTaskDelayUntil(&funcstarttime,uint16_t((1000*(uint32_t)60)/(uniman_running_conf.stepper_speed*(uint32_t)portTICK_PERIOD_MS)));

		for(int i=0;i<4;i++) {
			if((inmove[i]==1)&&(stepcmd[i].cursteps==stepcmd[i].tarsteps)) {
				
				uint16_t posrec=0;
				switch (i) {
					case 0:
					encoder1.readpos(&posrec);
					break;
					
					case 1:
					encoder2.readpos(&posrec);
					break;
					
					case 2:
					encoder3.readpos(&posrec);
					break;
					
					case 3:
					encoder4.readpos(&posrec);
					break;
				}
				
				uint16_t targetfind=abs(2048-stepcmd[i].tarenc);
				uint16_t encfind=abs(2048-posrec);
				
				
				csp_log_info("Stepper No %d Target %d Actual %d \n",i+1,stepcmd[i].tarenc,posrec,targetfind,encfind);
				if(abs(targetfind-encfind)<=RETRYMARGIN) {
					//if position is correct
					csp_log_info("Command Success, Delta %d Retry %d \n",abs(targetfind-encfind),stepcmd[i].retry);
					stepcmd[i].retry=0;
					inmove[i]=0;
					} else {
					if(stepcmd[i].retry<RETRYMAX) {
						csp_log_warn("Command Fail, Retry, Delta %d Retry %d\n",abs(targetfind-encfind),stepcmd[i].retry);

						//TODO - take into account going past setpoint, not jsut being less than it
						
						gs_fin_positions_t tempos;				
						for (int a=0; a<4;a++) {
							switch (a) {
								case 0:
									if(a==i){
										tempos.pos_fin_a=uniman_status.pos_set_points.pos_fin_a;
									} else {
										tempos.pos_fin_a=60000;
									}
								break;
								case 1:
									if(a==i){
										tempos.pos_fin_b=uniman_status.pos_set_points.pos_fin_b;
									} else {
										tempos.pos_fin_b=60000;
									}
								break;
								case 2:
									if(a==i){
										tempos.pos_fin_c=uniman_status.pos_set_points.pos_fin_c;
									} else {
										tempos.pos_fin_c=60000;
									}
								break;
								case 3:
									if(a==i){
										tempos.pos_fin_d=uniman_status.pos_set_points.pos_fin_d;
									} else {
										tempos.pos_fin_d=60000;
									}
								break;
							}	
							
						}
						
					
						stepcmd[i].retry++;
						//printf("1 %d 2 %d 3 %d 4 %d\n",tempos.pos_fin_a,tempos.pos_fin_b,tempos.pos_fin_c,tempos.pos_fin_d);
						set_fin_pos_ns(&tempos);
						} else {
						
						csp_log_error("Command Fail, Delta %d Retry %d\n",abs(targetfind-encfind),stepcmd[i].retry);
						stepcmd[i].retry=0;
						inmove[i]=0;
					}
				}
			}
		}
		
		if((inmove[0]+inmove[1]+inmove[2]+inmove[3])==0)  {
			uniman_status.mode=GS_FIN_MODE_CUSTOM;
			oversteps=BASEOVERSTEPS;
		}
		
		
// 		get_fin_status(&uniman_status);
// 		printf("pos %d %d %d %d\n",uniman_status.encoder_pos.pos_fin_a,uniman_status.encoder_pos.pos_fin_b,uniman_status.encoder_pos.pos_fin_c,uniman_status.encoder_pos.pos_fin_d);	
// 		uint16_t enctestar[4]= {0,0,0,0};
// 		encoder1.readpos(&enctestar[0]);
// 		encoder2.readpos(&enctestar[1]);
// 		encoder3.readpos(&enctestar[2]);
// 		encoder4.readpos(&enctestar[3]);
// 		printf("pos from enc %d %d %d %d\n",enctestar[0],enctestar[1],enctestar[2],enctestar[3]);
// 		
	}
	
	vTaskSuspend(NULL);
}




gs_fin_cmd_error_t init_server(void) {
	
	
	gs_fin_cmd_error_t error=FIN_CMD_OK;
	
	//I2C_init();
	printf("hello");
	encoder1.init();
	encoder2.init();
	encoder3.init();
	encoder4.init();

	
	setup_temp_sensors();
	
	uniman_stepper_q = csp_queue_create(STEPPER_QUEUE_LENGTH,sizeof(uint16_t));
	if (uniman_stepper_q==NULL) error =FIN_CMD_FAIL;
		
		
	// stepper q layout
	// [15:14] Which stepper, 00 is stepper 1, 01 is stepper 2 etc.
	// [13] direction
	// [12:0] no of steps
		

		
	running_conf_EEPROM.begin(EEPROM_RUN_CONF_ADD,2,sizeof(uniman_running_conf),&uniman_running_conf);

	gs_fin_positions_t temp = {
		.pos_fin_a=0,
		.pos_fin_b=0,
		.pos_fin_c=0,
		.pos_fin_d=0
	};
		

	stepper1.enstep();
	stepper2.enstep();
	stepper1.updateconfig(&uniman_fstep_conf,&uniman_running_conf);
	stepper2.updateconfig(&uniman_fstep_conf,&uniman_running_conf);
	//for(int i=0;i<100;i++){
	stepper1.dirfunc(0,0);
	stepper1.dirfunc(1,0);
	stepper1.stepfunc(0);
	stepper1.stepfunc(1);
	stepper2.stepfunc(0);
	stepper2.stepfunc(1);
	_delay_ms(20);
	stepper1.dirfunc(0,1);
	stepper1.dirfunc(1,1);
	stepper1.stepfunc(0);
	stepper1.stepfunc(1);
	stepper2.stepfunc(0);
	stepper2.stepfunc(1);
	_delay_us(100);
	//}
	stepper1.updateconfig(&uniman_step1_conf,&uniman_running_conf);
	stepper2.updateconfig(&uniman_step2_conf,&uniman_running_conf);
/*	save_fin_config();*/

	if(load_fin_config()) error=FIN_CMD_FAIL;

	print_conf(&uniman_running_conf);
		
	last_pos_rec.begin(EEPROM_LAST_POS_REC,8,sizeof(temp),&temp);

	last_pos_rec.read(&temp);
	set_fin_pos_ns(&temp);

		if(csp_thread_create(task_server, "SERVER", 270, NULL, 2, &handle_server)) error=FIN_CMD_FAIL;
	
	if(csp_thread_create(task_stepper, "STEP",configMINIMAL_STACK_SIZE+140, NULL, 1, NULL)) error=FIN_CMD_FAIL;


	
	return error;
	
	
	//TODO - also remember to initalise the status frame
	
}

uint8_t step_config_concat(uniman_ustep_mode_t a, uniman_invert_t b) {return a|b;};

gs_fin_cmd_error_t process_config(gs_fin_config_t * confin) {
	//get invert data
	
	gs_fin_cmd_error_t error=FIN_CMD_OK;
	
	uniman_running_conf=*confin;
	
	uniman_step1_conf.GCONF=STEPPER_GCONF_DATA(	(uniman_running_conf.stepper_config&(STEPPER_INVA_1))>>4	,(uniman_running_conf.stepper_config&(STEPPER_INVB_1))>>5);
	uniman_step2_conf.GCONF=STEPPER_GCONF_DATA(	(uniman_running_conf.stepper_config&(STEPPER_INVC_1))>>6	,(uniman_running_conf.stepper_config&(STEPPER_INVD_1))>>7);
	
	//get step data
	//if((uniman_running_conf.stepper_config&0x0F)!=0) { //if manually setting ustep setting, rahter than auto
	// 		uniman_step1_conf.CHOPCONF=(STEPPER_CHOPCONF_DEFAULT&~(0x0F000000))| (((uint32_t)(9-uniman_running_conf.stepper_config))<<24);
	// 		uniman_step2_conf.CHOPCONF=(STEPPER_CHOPCONF_DEFAULT&~(0x0F000000))| (((uint32_t)(9-uniman_running_conf.stepper_config))<<24);
	//
	
	// as discussed remote ustep is no longer allowed. code above would implement
	uniman_step1_conf.CHOPCONF=STEPPER_CHOPCONF_DEFAULT;
	uniman_step2_conf.CHOPCONF=STEPPER_CHOPCONF_DEFAULT;
	//}
	

	
	// do the currents, these translate directly
	uniman_step1_conf.IRUN=uniman_running_conf.stepper_irun;
	uniman_step2_conf.IRUN=uniman_running_conf.stepper_irun;
	
	uniman_step1_conf.IHOLD=uniman_running_conf.stepper_ihold;
	uniman_step2_conf.IHOLD=uniman_running_conf.stepper_ihold;
	
	if(stepper1.updateconfig(&uniman_step1_conf,confin)!=0) error = FIN_CMD_FAIL;
	if(stepper2.updateconfig(&uniman_step2_conf,confin)!=0) error = FIN_CMD_FAIL;
	
	return error;

}

gs_fin_cmd_error_t get_fin_config(gs_fin_config_t * conf) {
	*conf=uniman_running_conf;
	csp_log_info("Returning running config");
	return FIN_CMD_OK;
}

gs_fin_cmd_error_t set_fin_config(const gs_fin_config_t * conf) {
gs_fin_cmd_error_t error=FIN_CMD_OK;
	if(((conf->system_reset_encoder_zero)&(1<<4))!=0) {
		FORCERESET
	}
	
	if ((uniman_running_conf.system_reset_encoder_zero&(1<<3))!=0) {
		uniman_running_conf.system_reset_encoder_zero&=~(1<<3);
		if(encoder4.zeropos()!=0) error=FIN_CMD_FAIL;
	}
	
	if ((uniman_running_conf.system_reset_encoder_zero&(1<<2))!=0) {
		uniman_running_conf.system_reset_encoder_zero&=~(1<<2);
		if(encoder3.zeropos()!=0) error=FIN_CMD_FAIL;
	}
	
	if ((uniman_running_conf.system_reset_encoder_zero&(1<<1))!=0) {
		uniman_running_conf.system_reset_encoder_zero&=~(1<<1);
		if(encoder2.zeropos()!=0) error=FIN_CMD_FAIL;
	}
	
	if ((uniman_running_conf.system_reset_encoder_zero&(1<<0))!=0) {
		uniman_running_conf.system_reset_encoder_zero&=~(1<<0);
		if(encoder1.zeropos()!=0) error=FIN_CMD_FAIL;
	}
	
	uniman_running_conf=*conf;
	
	if(process_config(&uniman_running_conf)!=0) error=FIN_CMD_FAIL;
	csp_log_info("Setting running config");
	//TODO - action things like reset, zeroing
	
	
	return error;
}

gs_fin_cmd_error_t load_fin_config(void) {
	gs_fin_config_t temp;
	gs_fin_cmd_error_t error = FIN_CMD_OK;
	//print_conf(&uniman_running_conf);
	csp_log_info("Loading config from EEPROM");
	portENTER_CRITICAL();

	running_conf_EEPROM.read(&temp);
	//eeprom_read_block(&temp,(uint16_t*) EEPROM_RUN_CONF_ADD,sizeof(gs_fin_config_t));
	uniman_running_conf=temp;
	//print_conf(&uniman_running_conf);
	process_config(&uniman_running_conf);
	portEXIT_CRITICAL();
	csp_log_info("Error = %d",error);
	return FIN_CMD_OK;
}


gs_fin_cmd_error_t save_fin_config(void) {
	gs_fin_config_t temp;
	gs_fin_cmd_error_t error = FIN_CMD_OK;
	csp_log_info("Saving running config to EEPROM");
	portENTER_CRITICAL();
	running_conf_EEPROM.write(&uniman_running_conf);
	//eeprom_write_block(&uniman_running_conf,(uint16_t*) EEPROM_RUN_CONF_ADD,sizeof(gs_fin_config_t));

	running_conf_EEPROM.read(&temp);
	//eeprom_read_block(&temp,(uint16_t*) EEPROM_RUN_CONF_ADD,sizeof(gs_fin_config_t));

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
		printf("\nconf %x ",confin->stepper_config);
		printf("%x ",confin->stepper_ihold);
		printf("%x ",confin->stepper_irun);
		printf("%x ",confin->stepper_speed);
		printf("%x ",confin->system_reset_encoder_zero);
		printf("%x\n\n",confin->system_extra);
}

