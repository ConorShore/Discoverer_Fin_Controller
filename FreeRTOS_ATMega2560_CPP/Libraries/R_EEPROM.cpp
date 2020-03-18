 #include <stdio.h>
#include <avr/eeprom.h>
#include <csp/csp_crc32.h>
#include <R_EEPROM.h>

//this uses the CRC32 found in libcsp

	unsigned int R_EEPROM::addressspace[ADDRESSSPACESIZE];
	unsigned int R_EEPROM::blockreserve[ADDRESSSPACESIZE];
	 uint8_t R_EEPROM::initc;
	 
	 uint8_t R_EEPROM::initcheck(void) {
		 if(initc==0||initr==0) {
			return -1; 
		 } else {
			return 0; 
		 }
	 }


	
	int R_EEPROM::qsortcomp(uint16_t in1,uint16_t in2) {
		if (in1<in2) {
			return -1;
			} else if(in1==in2) {
			return 0;
		} else return 1;
		
	};
	
	uint8_t R_EEPROM::begin(uint16_t add, uint16_t blocksin, uint16_t sizeofdata, void * data) {
		if(initc==0) return -1;
		sizeofin=sizeofdata;
		uint8_t error = 0;
		uint16_t resamount=(uint16_t(sizeofin)+CRCSIZE+COUNTERSIZE)*blocksin;
		//printf("add %d resam %u sizeofin %d\n",add,resamount,sizeofin);
		for (int i=0;i<ADDRESSSPACESIZE;i++) {
					//printf("%x\n",addressspace[i]);
					
			if (addressspace[i]!=0xFFFF) { //array initalised to 0xFFFF, so if not this there is data here
				if (((add>=addressspace[i])&&(add<addressspace[i]+blockreserve[i]))||
						((add+resamount>addressspace[i])&&(add+resamount<=addressspace[i]+blockreserve[i]))) { 
							//does the address collide with preexisting data
					error = -1;
					return error;
				}
			} else {
				addressspace[i]=add;
				blockreserve[i]=resamount; 
				blocks=blocksin;
				break;
			}
			if(i==ADDRESSSPACESIZE-1) { //if theres no space left
				error=-1;
			
				return error;
			}
		}
		blocks=blocksin;
		startadd=add;
		curaddress=startadd;
		
		uint8_t uninit=1;
		
		for (int i=0;i<sizeofin+COUNTERSIZE+CRCSIZE;i++) {
			
			if(eeprom_read_byte((uint8_t *) startadd+i)!=0xFF) { //check to see if there's real data there
				uninit=0;
				break;
			}
		}
		initr=1;
		if(uninit) {
			printf("init eeprom area\n");
			if(write(data)!=0) return -1; // if theres no real data, write something there
		}
		
		
	
		
		return error;
	};
	
	uint8_t R_EEPROM::read(void * data) {
		
		if(initcheck()!=0) return -1;
		if(sizeofin>=BUFFSIZE) return -1;  //various sanity checks
		if(data==NULL) return -1;	
		
		uint8_t array[BUFFSIZE];

		eeprom_read_block(array,(int *) curaddress,sizeofin+CRCSIZE+COUNTERSIZE); //read the data
		
// 		for (int i=0;i<sizeofin+CRCSIZE+COUNTERSIZE;i++) {
// 			printf("%x ",array[i]);
// 		}
		
		crc_t crccalc=csp_crc32_memory(array,sizeofin+COUNTERSIZE); //calculate the crc of read data
		crc_t crcread=0;

		
		for(int i=0; i<CRCSIZE;i++) {

			crcread += (crc_t(array[sizeofin+COUNTERSIZE+i]) //get the crc read
									<<i*8);
		}
		
		//printf("\ncrc read %lx crc calc %lx\n",crcread,crccalc);
		
		if((crccalc-crcread)!=0) return -2; //compare crc's
		
		memcpy(data, array+COUNTERSIZE,sizeofin); // if everything checks out, pass that data out
		
		return 0;
		
		
	}
	
	R_EEPROM::R_EEPROM(void) {
		
		if(initc==0) {
			for(int i=0;i<ADDRESSSPACESIZE;i++) {
				addressspace[i] = 0xFFFF;
				blockreserve[i] = 0xFFFF;
			}
			initc=1;
		}
		initr=0;
		blockpoint=0;
	}
	
	uint8_t R_EEPROM::write(const void * data) {
		
		if(initcheck()!=0) return -1;
		if(sizeofin>=BUFFSIZE) return -1; //sanity checks
		if(data==NULL) return -1;
		
		uint8_t array[BUFFSIZE];

		memcpy(array+COUNTERSIZE,data,sizeofin+COUNTERSIZE); //copy data to new array, leaving room for counter
				
		uint8_t countbuff[COUNTERSIZE];
		count_t readcount=0;
		
		//printf("start add %u cur add %u",startadd,curaddress);
		
		
		eeprom_read_block(countbuff,(int *) curaddress,COUNTERSIZE); //read current val
		
		for(int i=0; i<COUNTERSIZE;i++) {
			readcount |= (countbuff[i]<<i*8);
		}
		
		count_t comp=0;
		comp=~comp;
		
		if(readcount==comp) { //this checks if area is initalised, if not start the count
			readcount=0;
		} else if(readcount>=MAXWRITES) {
			if(incrementblock()!=0) return -1; //if we need to increment the block, go ahead
		
			return write(data); //write data to new block
		}
		
	
		readcount++;
		memcpy(array,&readcount,COUNTERSIZE);

		crc_t crc=csp_crc32_memory(array,sizeofin+COUNTERSIZE); //calculate the crc
		
		for (int i=0;i<CRCSIZE;i++) {
			array[sizeofin+COUNTERSIZE+i]=(crc&(0xFFL<<i*8))>>i*8;
		}
		
// 		for (int i=0;i<sizeofin+CRCSIZE+COUNTERSIZE;i++) {
// 			printf("%x ",array[i]);
// 		}
// 		
		eeprom_write_block(array,(int *) curaddress,sizeofin+CRCSIZE+COUNTERSIZE);
				
				return 0;

		
	}

	
	uint8_t R_EEPROM::incrementblock(void) {
		if(blockpoint+1>=blocks) return -1;
		blockpoint++;
		printf("block incr\n");
		curaddress=startadd+(sizeofin*blockpoint);
		return 0;
	}

	
	