/* Copyright (c) 2013-2019 GomSpace A/S. All rights reserved. */
/**
   @file

   Fin controller server (implemented on Fin Controller Board)
*/

//csp_thread_handle_t handle_seraver;

#include <stdio.h>
#include "csp.h"
#include "csp_endian.h"
#include "csp_thread.h"
#include "fin.h"
#include "fin_server.h"
#include "spi.h"


csp_thread_handle_t handle_server;


//stepper motor defines, mostly address but some setup stuff
#define	STEPPER_GCONF_ADD 0x00
#define STEPPER_GCONF_DATA(a,b) 0x006 | a<<8 | b<<9 //a or b =1 will invert stepper dir, 0 wont
#define STEPPER_GSTAT 0x01
#define STEPPER_ISET1_ADD 0x30 // for setting currents to steppers
#define STEPPER_ISET2_ADD 0x50
#define STEPPER_MSCNT1_ADD 0x6A //microstep count
#define STEPPER_MSCNT2_ADD 0x7A
#define STEPPER_MSCURACT1_ADD 0x6B
#define STEPPER_MSCURACT2_ADD 0x7B
#define STEPPER_CHOPCONF1_ADD Ox6C
#define STEPPER_CHOPCONF2_ADD Ox7C

#define STEPPER_DEFAULT_IHOLD 31
#define STEPPER_DEFAULT_IRUN 31


typedef struct uniman_step_config {
	uint16_t	GCONF;
	uint8_t		IHOLD;
	uint8_t		IRUN;
	uint32_t	CHOPCONF;	 
}uniman_step_config_t;


uniman_step_config_t uniman_step1_conf = {
	.GCONF=STEPPER_GCONF_DATA(0,0),
	.IHOLD=STEPPER_DEFAULT_IHOLD,
	.IRUN=STEPPER_DEFAULT_IRUN,
	.CHOPCONF=0x08010008
};

uniman_step_config_t uniman_step2_conf = {
	.GCONF=STEPPER_GCONF_DATA(0,0),
	.IHOLD=STEPPER_DEFAULT_IHOLD,
	.IRUN=STEPPER_DEFAULT_IRUN,
	.CHOPCONF=0x08010008
};

//functions for controlling certain pins of steppers

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


class tmc2041 {
	
	private:
	uniman_step_config_t config;
	void (*csinit)();
	void (*cson)();
	void (*csoff)();
	void (*enstep)();
	void (*disstep)();
	
	public:
	tmc2041(void (csinitin()),void (csonin()),void (csoffin()),void (enstepin()), void (disstepin()),
	uniman_step_config_t configin) : csinit(csinitin), cson(csonin), csoff(csoffin) , enstep(enstepin),
	disstep(disstepin) {
		
		config=configin;
		csinit();
		
	};
	gs_fin_cmd_error_t set_speed(void);
	gs_fin_cmd_error_t set_pos1(void);
	gs_fin_cmd_error_t set_pos2(void);
	
	
	
	
	};
	



// defines for temp sensors
	#define TEMP_MASK1 0x03
	#define TEMP_MASK2 0x02
	#define TEMP_MASK3 0x01
	#define TEMP_MASK4 0x00
	
	#define TEMP_CAL_T0 273.15
	#define TEMP_CAL_T1 298.15
	#define TEMP_CAL_T2
	#define TEMP_CAL_RT1 10000.0
	#define TEMP_CAL_RT2
	#define TEMP_CAL_BETA (log(TEMP_CAL_RT1/TEMP_CAL_RT2))/((1/TEMP_CAL_T1)-(1/TEMP_CAL_T2))
	#define TEMP_RES_VAL 10000.0
	#define TEMP_RES_INF TEMP_RES_VAL*(exp((-TEMP_CAL_BETA)/TEMP_CAL_T0))
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
		//tempstore=(TEMP_RES_VAL*ADC)/(1024-ADC);    //sort this out
		//*(array)=(uint16_t)(TEMP_CAL_BETA/(log((tempstore)/TEMP_RES_INF));
		
		ADMUX=(ADMUX&~(0x0F))|TEMP_MASK2;
		ADCSRA|=(1<<ADSC);
		while(ADCSRA&(1<<ADSC));
		*(array+1)=ADC;
		
		ADMUX=(ADMUX&~(0x0F))|TEMP_MASK3;
		ADCSRA|=(1<<ADSC);
		while(ADCSRA&(1<<ADSC));
		*(array+2)=ADC;
		
		ADMUX=(ADMUX&~(0x0F))|TEMP_MASK4;
		ADCSRA|=(1<<ADSC);
		while(ADCSRA&(1<<ADSC));
		*(array+3)=ADC;
		portEXIT_CRITICAL();

	
		
	}


static void process_fin_cmd(csp_conn_t * conn, csp_packet_t * packet)
{
    uint16_t reply_length = 1;
    uint8_t cmd_id = packet->data[0]; // First byte is command ID

    switch(cmd_id) {
        case GS_FIN_CMD_STATUS: {
            gs_fin_status_t status;

            /* Get status internally */
            int8_t error = get_fin_status(&status);

            /* Set error code in response */
            packet->data[0] = error;

            if (error == FIN_CMD_OK) {
                /* Copy status to response buffer */
                memcpy(&packet->data[1], &status, sizeof(status));
                reply_length += sizeof(status);
            }
            break;
        }

        case GS_FIN_CMD_SET_CUSTOM_POSITION: {
            gs_fin_positions_t pos;

            /* Copy received positions (angles) to struct */
            memcpy(&pos, &packet->data[1], sizeof(pos));

            /* Ensure correct endianness */
            uint16_t * pos_array = (uint16_t *) &pos; //(void *)
            for (int i = 0; i < 4; i++) {
                pos_array[i] = csp_ntoh16(pos_array[i]);
            }
            /* Pass positions to internal layer */
            int8_t error = set_fin_pos(&pos);

            /* Set error code in response */
            packet->data[0] = error;
            break;
        }

        case GS_FIN_CMD_SET_MAX_DRAG: {
            /* Pass command to internal layer */
            int8_t error = set_max_drag();

            /* Set error code in response */
            packet->data[0] = error;
            break;
        }

        case GS_FIN_CMD_SET_MIN_DRAG: {
            /* Pass command to internal layer */
            int8_t error = set_min_drag();

            /* Set error code in response */
            packet->data[0] = error;
            break;
        }

        default: {
            /* Set error code in response to no command found */
            packet->data[0] = FIN_CMD_NOT_FOUND;
            break;
        }
    }

    /* Send response */
    packet->length = reply_length;
    if (!csp_send(conn, packet, 0)) {
        csp_buffer_free(packet);
    }
}

/**
   CSP server task.

   Handles generic CSP cmds and fin controller cmds.
   CSP must be enabled prior to creation of this task.
 */
CSP_DEFINE_TASK(task_server)
{
    /* Create socket without any socket options */
    csp_socket_t *sock = csp_socket(CSP_SO_NONE);

    /* Bind all ports to socket */
    csp_bind(sock, CSP_ANY);

    /* Create 10 connections backlog queue */
    csp_listen(sock, 10);

    /* Pointer to current connection and packet */
    csp_conn_t * conn;
    csp_packet_t * packet;

    /* Process incoming connections */
    while (1) {
csp_log_info("%s %d\n",pcTaskGetName(NULL),uxTaskGetStackHighWaterMark2(NULL));
        /* Wait for connection, 10000 ms timeout */
        if ((conn = csp_accept(sock, 10000)) == NULL) {
            continue;
        }

        /* Read packets */
        while ((packet = csp_read(conn, 0)) != NULL) { //was 100 in my code
            switch (csp_conn_dport(conn)) {
                case GS_FIN_CONTROL_PORT:
                    /* Process fin controller packet here */
                    process_fin_cmd(conn, packet);
                    break;

                default:
                    /* Let the service handler reply pings, buffer use, etc. */
                    csp_service_handler(conn, packet);
                    break;
            }
        }

        /* Close current connection, and handle next */
        csp_close(conn);

    }

    return CSP_TASK_RETURN;
}



gs_fin_cmd_error_t init_server(void) {
		
	tmc2041 stepper1(&stepper_1_cs_init,&stepper_1_cson,&stepper_1_csoff,&stepper_1_en, &stepper_1_dis,  uniman_step1_conf);
	tmc2041 stepper2(&stepper_2_cs_init,&stepper_2_cson,&stepper_2_csoff,&stepper_2_en, &stepper_2_dis,  uniman_step2_conf);	
	
	
	if(!csp_thread_create(task_server, "SERVER", 270, NULL, 2, &handle_server)) {
		return FIN_CMD_OK;
	} else {
		return FIN_CMD_FAIL;
		
	}

	//should also initalise other things such as temp sensors and steppers here
	
	//also remember to initalise the status frame
	
}


