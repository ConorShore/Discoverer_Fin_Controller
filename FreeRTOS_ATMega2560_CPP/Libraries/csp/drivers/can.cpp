#include <stdint.h>
#include <stdio.h>

#include <csp/csp.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/drivers/can.h>
#include <SPI/spi.h>
#include <arduino-mcp2515-master/mcp2515.h>
#include <csp/arch/csp_queue.h>
#include <csp/csp_debug.h>
#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/task.h>

//static csp_queue_handle_t csp_can_rx_queue;



MCP2515 mcp2515;
void canisrinit(void);
#define errorfunc(s,er) if(s!=MCP2515::ERROR_OK) {er=-1;}
	
	
	#define MAX_DELAY 262
	#define MAX_CONC_FRAME (CSP_CAN_MTU/8)+1
	
	
#define RETRY_NO 8



int can_init(uint32_t id, uint32_t mask, struct csp_can_config *conf) {
	int error = 0;

		SPI.begin();
			errorfunc(mcp2515.reset(),error);
			errorfunc(mcp2515.setBitrate(CAN_1000KBPS),error);
			//printf(" mask = %lx",mask);
			//while(1);
			errorfunc(mcp2515.setFilterMask(MCP2515::MASK0,1,mask),error);
 			errorfunc(mcp2515.setFilterMask(MCP2515::MASK1,1,mask),error);
 			errorfunc(mcp2515.setFilter(MCP2515::RXF0,1,id),error);
 			errorfunc(mcp2515.setFilter(MCP2515::RXF2,1,id),error);
			errorfunc(mcp2515.setNormalMode(),error);
			

			


			
	return error;
}


int can_send(can_id_t id, uint8_t * data, uint8_t dlc){
	int error=0;
	const MCP2515::TXBn tx0 =MCP2515::TXB0;
	struct can_frame canMsg;
	portENTER_CRITICAL();
	canMsg.can_id=id | CAN_EFF_FLAG;
	canMsg.can_dlc=dlc;
	for (int i=0;i<dlc;i++) {
	
		canMsg.data[i]=*(data+i);
	}
	
	for(int i =0;i<RETRY_NO;i++) {
	if(mcp2515.checktxava()==MCP2515::ERROR_OK){
		_delay_us(100); //magic number because even if the MCP2515 reports complete tx it can ignore if data written too quickly
			errorfunc(mcp2515.sendMessage(tx0,&canMsg),error);
			break;
	}
	
	}
			//_delay_us(500);
	//timeoutstart(10000);
	
	/*while(!(timeoutcheck())){
		if(mcp2515.checktxava()==MCP2515::ERROR_OK){
			errorfunc(mcp2515.sendMessage(tx0,&canMsg),error);
		
		break;
		}
	}*/
	
	
	portEXIT_CRITICAL();
	//csp_log_info("%d\n",ctr);
	if(error!=0) {csp_log_error("can tx error");}
	return error;
	
}



void CanRxFunc(void* pvParameters) {
	

	DDRD&=~(1<<PD2);
	EICRA|=(2<<ISC20);
	EIMSK|=(1<<INT2);
	
	mcp2515.clearInterrupts();
	//DDRB&=~(1<<PB5);
	//PCICR|=(1<<PCIE0);
	//PCMSK0|=(1<<PCINT5);
			 static can_frame_t frameo[MAX_CONC_FRAME];
			 static struct can_frame frame[MAX_CONC_FRAME];
	
	

	for(;;){
		

		mcp2515.clearInterrupts();
		//csp_log_info("%s %d\n",pcTaskGetName(NULL),uxTaskGetStackHighWaterMark2(NULL));
		vTaskSuspend(NULL);
		
			uint8_t count=0;
			uint8_t remain =MAX_CONC_FRAME-1;
			int error=0;
			int errorb=0;
			int dlcsum=0;
			portENTER_CRITICAL();
			timeoutstart(MAX_DELAY); // 256*n/16M for time
			while(!(timeoutcheck())){ //while the time hasn't elapsed
			
				if(mcp2515.readMessage(&frame[count]) == MCP2515::ERROR_OK&&error==0) { //check for a new message and store in &frame
					//dlcsum+=frame[count].can_dlc;
					 uint8_t begin = (((frame[count].can_id)>>18)&0x01); //get the begin flag
						//csp_log_info("t= %lx\n",test);
						if (begin!=1&&count==0) { //if it is a begin frame
							remain = ((uint8_t)((frame[count].can_id >> 10) & 0xFF)); //get the remain from it
							// printf("h%d",remain);
							if(remain>(MAX_CONC_FRAME-1)) { //if the frames coming in are too big they will be ignored
								error=1;
							}

					
						} else if (count==0) {errorb=1; break;} // if its the first frame received and not a begin frame, break
						if (error==1) {count=1;}

						count++; // increment number of messages received
						timeoutreset(); //if there is a new message, reset the timer
					}		
					if (count>remain){break;} // this is used to break out of the timeout if we've received all frames or if there's too many
			}
			portEXIT_CRITICAL();
			//csp_log_info("DLC = %d",dlcsum);
			if(error!=0||errorb!=0) { // if too many frames come it, it wont process them. saves time handling these useless requests
 				csp_log_error("Too Many Frames or wrong order");
			
			} else {
			
			//csp_log_info("CAN Rec = %d",count); 
			for (int i=0;i<count;i++){
				uint8_t pos = count-((uint8_t)((frame[i].can_id >> 10) & 0xFF))-1;
				if(pos>=count) {
					csp_log_error("impossible pos %d	count = %d\r\n",pos,count);
					csp_log_error("Id = %ux\r\n",frame[i].can_id);
					csp_log_error("DLC = %ux\r\n",frame[i].can_dlc);
					for (int a=0;a<frame[i].can_dlc;a++) {
						csp_log_error("Data %d = %x",a,frame[i].data[a]);
					}
				}
				
				
				//get remain from CFP header and use it for position of data
				// this is done because there is no mechanism in the MCP2515 to determine which rx buffer was filled
				//first, so possibility of reading out of order information
				

				
				//csp_log_info("pos = %d	count=%d",pos,count);
				
				
				// next part translates from libraries can frame to libcsp's
							frameo[pos].id=frame[i].can_id;
							//frameo.id&=~(0xE0000000); //this was here ebcause for some reason first 3 bits were 1's of id
					frameo[pos].dlc=frame[i].can_dlc;
					for (uint8_t a=0;a<frame[i].can_dlc;a++) {
						frameo[pos].data[a]=frame[i].data[a];
					}
			}
			
			/*for (int i=0;i<count;i++){
					csp_log_info("%08lx %d",frameo[count].id,frameo[count].dlc);
					for(int a=0;a<frameo[count].dlc;a++) {
						csp_log_info(" %x",frameo[count].data[a]);
					}
					csp_log_info("\n");
				
			}*/
			portENTER_CRITICAL();
			for (int i=0;i<count;i++){ //send of the frames to be processed
				if(i==count-1){
					
					if(csp_can_rx_frame(&frameo[i],"CAN")!=0) {csp_log_reset("queue fail");}
					
					} else {
					if(csp_can_rx_frame(&frameo[i],NULL)!=0) {csp_log_reset("queue fail");}
					}
					
					
				
			}
			portEXIT_CRITICAL();
			
			for(int i=0;i<MAX_CONC_FRAME;i++){
				frameo[i].id=0;
				frame[i].can_id=0;
				frameo[i].dlc=0;
				frame[i].can_dlc=0;
				for (int a=0;a<8;a++) {
					frameo[i].data[a]=0;
					frame[i].data[a]=0;
				}
			}
					}
			//printf("%s %d\n",pcTaskGetName(NULL),uxTaskGetStackHighWaterMark2(NULL));
			

		//uint8_t irq = mcp2515.getInterrupts();
		
		/*while(mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
							frameo.id=frame.can_id;
							//frameo.id&=~(0xE0000000);
					frameo.dlc=frame.can_dlc;
					for (uint8_t i=0;i<frame.can_dlc;i++) {
						frameo.data[i]=frame.data[i];
					}
					csp_log_info("%08lx %d",frameo.id,frameo.dlc);
					for(int i=0;i<frameo.dlc;i++) {
						csp_log_info(" %x",frameo.data[i]);
					}
				if(csp_can_rx_frame(&frameo,NULL)!=0) {csp_log_error("queue fail");}
				}*/

		
		
		
		
		}
		
		vTaskDelete(NULL);
  	
		}



/*if (irq & MCP2515::CANINTF_RX0IF) { //irq & MCP2515::CANINTF_RX0IF
			if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
				// frame contains received from RXB0 message
									frameo.id=frame.can_id;
					frameo.dlc=frame.can_dlc;
					for (uint8_t i=0;i<frame.can_dlc;i++) {
						frameo.data[i]=frame.data[i];
					}
					csp_log_error("%08lx %d %c %c\n",frameo.id,frameo.dlc,frameo.data[0],frameo.data[1]);
				if(csp_can_rx_frame(&frameo,NULL)!=0) {csp_log_error("queue fail");}
					
					
			}
		}

		if (irq & MCP2515::CANINTF_RX1IF) {
			if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) {
				// frame contains received from RXB1 message
							frameo.id=frame.can_id;
					frameo.dlc=frame.can_dlc;
					for (uint8_t i=0;i<frame.can_dlc;i++) {
						frameo.data[i]=frame.data[i];
					}
					csp_log_error("%08lx %d %c %c\n",frameo.id,frameo.dlc,frameo.data[0],frameo.data[1]);
				if(csp_can_rx_frame(&frameo,NULL)!=0) {csp_log_error("queue fail");}
				//if(csp_queue_enqueue(&csp_can_rx_queue,&frame,NULL)) {csp_log_error("queue fail");}
			}*/