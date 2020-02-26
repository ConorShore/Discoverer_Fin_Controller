#ifndef SRC_SOAR_FIN_FIN_H_
#define SRC_SOAR_FIN_FIN_H_
/* Copyright (c) 2013-2019 GomSpace A/S. All rights reserved. */
/**
   @file

   Fin controller interface API.
*/


#include <stdint.h>

/**
   CSP port for fin controller commands.
 */
#define GS_FIN_CONTROL_PORT     31

/**
   Internal fin controller modes/states
 */
typedef enum {
    /**
       Controller is in initialization mode.
    */
    GS_FIN_MODE_INIT = 0,
    /**
       Controller is moving the fins.
    */
    GS_FIN_MODE_MOVING = 1,
    /**
       Fins are at the custom set position.
    */
    GS_FIN_MODE_CUSTOM = 2,
    /**
       Fins are at the maximum drag position.
    */
    GS_FIN_MODE_MAX_DRAG = 3,
    /**
       Fins are at minimum drag position.
    */
    GS_FIN_MODE_MIN_DRAG = 4,
} gs_fin_mode_t;

/**
   Fin controller command ID's
 */
typedef enum {
    /**
       Get status/telemetry from fin controller.
    */
    GS_FIN_CMD_STATUS = 0,
    /**
       Set custom fin position at controller.
    */
    GS_FIN_CMD_SET_CUSTOM_POSITION = 1,
    /**
       Set fin position in maximum drag position.
    */
    GS_FIN_CMD_SET_MAX_DRAG = 2,
    /**
       Set fin position in minimum drag position.
    */
    GS_FIN_CMD_SET_MIN_DRAG = 3,
	
	/**
	Get config parameters
	*/
	UNIMAN_FIN_CFG_GET = 4,
	
	/**
	Set config parameters
	*/
	
	UNIMAN_FIN_CFG_SET = 5,
	
	/**
	Save current config parameters in ram to EEPROM
	*/
	
	UNIMAN_FIN_CFG_SAVE = 6
	
} gs_fin_commands_t;

/**
   Fin controller command error codes.
 */
typedef enum {
    /**
       CMD success
    */
    FIN_CMD_OK = 0,
    /**
       CMD failure
    */
    FIN_CMD_FAIL = -1,
    /**
       CMD not found
    */
    FIN_CMD_NOT_FOUND= -2,
} gs_fin_cmd_error_t;

/**
   Fin positions
   Note it is 1 byte aligned as this struct goes directly on the network.
 */
typedef struct  __attribute__((packed, aligned(1))) gs_fin_positions {
    /**
       Position of fin A in deci degrees (1/10 degree steps).
     */
    uint16_t pos_fin_a;
    /**
       Position of fin B in deci degrees (1/10 degree steps).
     */
    uint16_t pos_fin_b;
    /**
       Position of fin C in deci degrees (1/10 degree steps).
     */
    uint16_t pos_fin_c;
    /**
       Position of fin D in deci degrees (1/10 degree steps).
     */
    uint16_t pos_fin_d;
} gs_fin_positions_t ;

/**
   Fin status/telemetry
   Note it is 1 byte aligned as this struct goes directly on the network.
 */
typedef struct __attribute__((packed, aligned(1))) gs_fin_status {
    /**
       Current position set points.
     */
    gs_fin_positions_t pos_set_points;
    /**
       Current position measured from encoders (actual position).
     */
    gs_fin_positions_t encoder_pos;
    /**
       Temperature measurements [K]
     */
    uint16_t temperatures[4];
    /**
       Current measurements [mA]
     */
    uint16_t currents[4];
    /**
       Current mode, see @gs_fin_mode_t
     */
    gs_fin_mode_t mode;
    /**
       Internal status code
     */
    uint8_t status_code;
} gs_fin_status_t ;

typedef enum {
	
	//code for selecting uStep mode
	//this will apply to all stepper motors
	//auto means the controller will decide
	// any other setting will force microstepping in that mode
	
	STEPPER_USTEP_AUTO = 0,
	STEPPER_USTEP_FULL	= 1,
	STEPPER_USTEP_1_2 = 2,
	STEPPER_USTEP_1_4 = 3,
	STEPPER_USTEP_1_8 = 4,
	STEPPER_USTEP_1_16 = 5,
	STEPPER_USTEP_1_32 = 6,
	STEPPER_USTEP_1_64 = 7,
	
}uniman_ustep_mode_t;

typedef enum {
	
	// for setting inverts for each steppers
	
	STEPPER_INVA_0 = (0<<4),
	STEPPER_INVA_1 = (1<<4),
	STEPPER_INVB_0 = (0<<5),
	STEPPER_INVB_1 = (1<<5),
	STEPPER_INVC_0 = (0<<6),
	STEPPER_INVC_1 = (1<<6),
	STEPPER_INVD_0 = (0<<7),
	STEPPER_INVD_1 = (1<<7),
	
}uniman_invert_t;

// 1=A=y+, 2=B=x+, 3=C=y-, 4=D=x-



typedef struct __attribute__((packed, aligned(1))) uniman_fin_config {
	
	uint8_t stepper_config; // contains the step mode and invert statuses for steppers.
							// uses uniman_ustep_mode_t or'd with uniman_ustep_invert_t.
	uint8_t stepper_ihold; //stepper hold current. as per tmc2041 data sheet value from 0-31
	uint8_t stepper_irun; //stepper run current. as per tmc2041 data sheet value from 0-31
	uint16_t stepper_speed; //speed to move for custom positions moves. defined as full steps per minute. typ =60
	uint8_t system_reset_encoder_zero; //[7:5]reserved,[4]reset controller,[3:0]zero encoder D-A
	uint8_t system_extra; // [7:0] reserved
	

	}uniman_fin_config_t;

#endif /* SRC_SOAR_FIN_FIN_H_ */
