



#include <stdlib.h>
#include <string.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>


#include <SPI/spi.h>
#include <arduino-mcp2515-master/mcp2515.h>

//#define MSGMAX 50 // maximum CSP message size

extern "C" {


/* Scheduler include files. */
#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/task.h>
#include <FreeRTOS/croutine.h>

// CSP files

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <csp/csp.h>

/* Using un-exported header file.
 * This is allowed since we are still in libcsp */
#include <csp/arch/csp_thread.h>

//uart stuff for serial0

#include <avr_usart_init.h>



#include <csp/csp_rtable.h>





}
#include <fin.h>
#include <fin_server.h>

#include <csp/drivers/can.h>
#include <csp/interfaces/csp_if_can.h>

	//#include <avr/fuse.h>

#define FUSES __fuse_t __fuse FUSEMEM

FUSES {.low = 0xBF, .high=0x01,.extended = 0xFF};









//static char out[10]; //used for as placeholder for itoa function

#define BAUD 500000 //UART baud







/** Example defines */
#define MY_ADDRESS  6			// Address of local CSP node
#define CAN_CSP_ADDRESS 6		//
#define OBC_ADDRESS 1
#define MY_PORT		10			// Port to send test traffic to



/* Priority definitions for most of the tasks in the demo application.  Some
tasks just use the idle priority. */
#define mainLED_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )

/* Baud rate used by the serial port tasks. */
//#define mainCOM_TEST_BAUD_RATE			( ( unsigned long ) 38400 )

/* LED used by the serial port tasks.  This is toggled on each character Tx,
and mainCOM_TEST_LED + 1 is toggles on each character Rx. */
#define mainCOM_TEST_LED				( 4 )

/* LED that is toggled by the check task.  The check task periodically checks
that all the other tasks are operating without error.  If no errors are found
the LED is toggled.  If an error is found at any time the LED is never toggles
again. */
#define mainCHECK_TASK_LED				( 7 )

/* The period between executions of the check task. */
#define mainCHECK_PERIOD				( ( TickType_t ) 3000 / portTICK_PERIOD_MS  )

/* An address in the EEPROM used to count resets.  This is used to check that
the demo application is not unexpectedly resetting. */
#define mainRESET_COUNT_ADDRESS			( ( uint8_t * ) 0x50 )

/* The number of coroutines to create. */
#define mainNUM_FLASH_COROUTINES		( 3 )



static void TaskBlinkyellowLED(void* pvParameters);



// CSP prototypes
CSP_DEFINE_TASK(task_server);
CSP_DEFINE_TASK(task_client);
/*-----------------------------------------------------------*/

#include "fincont.h"

struct can_frame canMsg1;
struct can_frame canMsg2;

	csp_thread_handle_t handle_canrx; // handle for RX interrupt





	
	struct csp_can_config can_conf = {
		.bitrate = 1000000UL,
		.clock_speed = 10000000UL,
		.ifc="can0"
	};
	
	static void wdtr(void* pvParameters);
	

	

	


int main( void )
{
	usart_init(MYUBRR);
fdevopen( &usart_putchar_printf,0);


	
	wdt_enable(WDTO_500MS); // set watchdog up to reset if not called after 2s
	//pwrredinit();
	enableleds();






		int error=0;
		
		error+=csp_buffer_init(4, MSGMAX);

	
		// Init CSP with address MY_ADDRESS 
		error+=csp_init(MY_ADDRESS);
		
		error+=csp_can_init(0,&can_conf);
		
		error+=csp_route_set(CSP_DEFAULT_ROUTE, &csp_if_can, 0xFF);

		// Start router task with 500 word stack, OS task priority 1 		
		error+=csp_route_start_task(180, 1);
		
		
		
		error-=init_server();
		printf("%d",error);

		// Client 
		
		csp_thread_handle_t handle_client;
		error+=csp_thread_create(task_client, "CLIENT", 220, NULL, 1, &handle_client);
		
		//csp_log_reset("test");
		error+=csp_thread_create(CanRxFunc,"CANRX",180,NULL,3,&handle_canrx);
		
		if(error!=0) {
			contled(ONCON,2);
			csp_log_reset("Setup Error");
			//FORCERESET
		}

		//printf("Debug enabed\r\n");
		
		//csp_debug_toggle_level(CSP_INFO);
		//csp_debug_toggle_level(CSP_WARN);
		//csp_debug_toggle_level(CSP_ERROR);

		
		printf("Conn table\r\n");
		csp_conn_print_table();

		printf("Route table\r\n");
		csp_route_print_table();

		printf("Interfaces\r\n");
		csp_route_print_interfaces();
		
		
		xTaskCreate(wdtr,"WDT",configMINIMAL_STACK_SIZE,NULL,3,NULL);
		
		//while(1);
	 	 //xTaskCreate(TaskBlinkserial, "blinks", 128, NULL, 3, NULL);
	// usart_pstr_p(PSTR("LED init"),1);


	vTaskStartScheduler();
	
	

	return 0;
}
/*-----------------------------------------------------------*/


static void wdtr(void* pvParameters) {
	 wdt_reset();
	// wdt_disable();
	 wdt_enable(WDTO_500MS);
	 
	 int a =0;
	 
	for(;;) {
	wdt_reset();
	vTaskDelay(200/ portTICK_PERIOD_MS);
	contled(TOGCON,0);

	}
}

static void TaskBlinkyellowLED(void* pvParameters)
{	
	// set pin 7 of PORTB for output
	DDRB |= (1<<7);

	TickType_t xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{

		// LED on
		PORTB=(1<<PB7);
		vTaskDelayUntil(&xLastWakeTime, (1000/ portTICK_PERIOD_MS));

		// LED off

		PORTB=0;
		vTaskDelayUntil(&xLastWakeTime, (1000/ portTICK_PERIOD_MS));
		


	}

	vTaskDelete(NULL);
}



ISR(INT2_vect){
	//if((PORTB&PB5)>>PB5) return;
		
	PCIFR&=~(1<<PCIF0);
	//printf("ISR ENTER");
	//while(1);

	volatile BaseType_t xYieldRequired;

	 // Resume the suspended task.
	 xYieldRequired = xTaskResumeFromISR(handle_canrx);

	 if( xYieldRequired == pdTRUE )
	 {
		 // We should switch context so the ISR returns to a different task.
		 // NOTE:  How this is done depends on the port you are using.  Check
		 // the documentation and examples for your port.
		 taskYIELD();
	 }

}


//  old server task used previously
// 
// CSP_DEFINE_TASK(task_server) {
// 
// 	/* Create socket without any socket options */
// 	csp_socket_t *sock = csp_socket(CSP_SO_NONE);
// 	//usart_pstr_p(PSTR("server task init"),1);
// 	/* Bind all ports to socket */
// 	csp_bind(sock, CSP_ANY);
// 
// 	/* Create 10 connections backlog queue */
// 	csp_listen(sock, 10);
// 
// 	/* Pointer to current connection and packet */
// 	csp_conn_t *conn;
// 	csp_packet_t *packet;
// 
// 	/* Process incoming connections */
// 	for(;;) {
// 		csp_log_info("%s %d\n",pcTaskGetName(NULL),uxTaskGetStackHighWaterMark2(NULL));
// 
// 		/* Wait for connection, 10000 ms timeout */
// 		if ((conn = csp_accept(sock, 10000)) == NULL)
// 			continue;
// 
// 		/* Read packets. Timout is 100 ms */
// 		while ((packet = csp_read(conn, 100)) != NULL) {
// 			
// 			switch (csp_conn_dport(conn)) {
// 			case MY_PORT:
// 				/* Process packet here */
// 				printf("Packet received on MY_PORT: %s\r\n", (char *) packet->data);
// 				
// 				
// 				
// 				csp_buffer_free(packet);
// 				break;
// 
// 			default:
// 				/* Let the service handler reply pings, buffer use, etc. */
// 			
// 				csp_service_handler(conn, packet);
// 				break;
// 			}
// 		}
// 
// 		/* Close current connection, and handle next */
// 		csp_close(conn);
// 
// 	}
// 
// 	return CSP_TASK_RETURN;
// 
// }

CSP_DEFINE_TASK(task_client) {

	csp_packet_t * packet;
	csp_conn_t * conn;
	//usart_pstr_p(PSTR("client task init"),1);

	for(;;) {

		/**
		 * Try ping
		 */
	csp_log_info("%s %d\n",pcTaskGetName(NULL),uxTaskGetStackHighWaterMark2(NULL));
		csp_sleep_ms(1000);
		
		//int *sph = 0x3E;
		//volatile unsigned int *sph = (volatile unsigned int *)0x5E;
		//volatile unsigned int *spl = (volatile unsigned int *)0x5D;
		volatile unsigned int *sp=(volatile unsigned int *)SP;
		
		
		//uint16_t StackP = ((*(sph))<<8)+(*(spl));
		int StackP= *sp;
		csp_log_info("SP = 0x%04x\n",StackP);;
		int result = csp_ping(1, 100, 10, CSP_O_NONE);

		if (result==-1) {csp_log_error("Ping Failed\n");}
		csp_log_info("Ping result %d [ms]\r\n", result);

		csp_sleep_ms(1000);

		/**
		 * Try data packet to server
		 */

		

		/* Get packet buffer for data */
// 		packet = (csp_packet_t*)csp_buffer_get(MSGMAX);
// 	
// 		if (packet == NULL) {
// 			/* Could not get buffer element */
// 		
// 			printf("Failed to get buffer element\n");
// 			return CSP_TASK_RETURN;
// 		}
// 
// 		/* Connect to host HOST, port PORT with regular UDP-like protocol and 1000 ms timeout */
// 		conn = csp_connect(CSP_PRIO_NORM, 1, MY_PORT, 1000, CSP_O_NONE);
// 		if (conn == NULL) {
// 			/* Connect failed */
// 
// 			printf("Connection failed\n");
// 			/* Remember to free packet buffer */
// 			csp_buffer_free(packet);
// 			return CSP_TASK_RETURN;
// 		}
// 
// 		/* Copy dummy data to packet */
// 		char msg[] = "Hello World";
// 		itoa(xTaskGetTickCount(),out,10);
// 		strcpy((char *) packet->data, out);
// 
// 		/* Set packet length */
// 		packet->length = strlen(msg);
// 
// 		/* Send packet */
// 		if (!csp_send(conn, packet, 1000)) {
// 			/* Send failed */
// 		
// 			printf("Send failed\n");
// 			csp_buffer_free(packet);
// 		}
// 
// 		/* Close connection */
// 		csp_close(conn);

	}

	return CSP_TASK_RETURN;
}



ISR(BADISR_vect)
{
	//FORCERESET
 printf("Unex Vector");
	FORCERESET
}

