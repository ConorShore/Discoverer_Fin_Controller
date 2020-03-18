 #include <stdio.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <string.h>

#include <R_EEPROM.h>

//this implementation is copied and sightly modified to reduce memory usage
// originally comes from libcsp https://github.com/GomSpace/libcsp

static const uint32_t crc_tab[256] PROGMEM = {
		0x00000000, 0xF26B8303, 0xE13B70F7, 0x1350F3F4, 0xC79A971F, 0x35F1141C, 0x26A1E7E8, 0xD4CA64EB,
		0x8AD958CF, 0x78B2DBCC, 0x6BE22838, 0x9989AB3B, 0x4D43CFD0, 0xBF284CD3, 0xAC78BF27, 0x5E133C24,
		0x105EC76F, 0xE235446C, 0xF165B798, 0x030E349B, 0xD7C45070, 0x25AFD373, 0x36FF2087, 0xC494A384,
		0x9A879FA0, 0x68EC1CA3, 0x7BBCEF57, 0x89D76C54, 0x5D1D08BF, 0xAF768BBC, 0xBC267848, 0x4E4DFB4B,
		0x20BD8EDE, 0xD2D60DDD, 0xC186FE29, 0x33ED7D2A, 0xE72719C1, 0x154C9AC2, 0x061C6936, 0xF477EA35,
		0xAA64D611,	0x580F5512, 0x4B5FA6E6, 0xB93425E5, 0x6DFE410E, 0x9F95C20D, 0x8CC531F9, 0x7EAEB2FA,
		0x30E349B1,	0xC288CAB2, 0xD1D83946, 0x23B3BA45, 0xF779DEAE, 0x05125DAD, 0x1642AE59, 0xE4292D5A,
		0xBA3A117E,	0x4851927D, 0x5B016189, 0xA96AE28A, 0x7DA08661, 0x8FCB0562, 0x9C9BF696, 0x6EF07595,
		0x417B1DBC,	0xB3109EBF, 0xA0406D4B, 0x522BEE48, 0x86E18AA3, 0x748A09A0, 0x67DAFA54, 0x95B17957,
		0xCBA24573,	0x39C9C670, 0x2A993584, 0xD8F2B687, 0x0C38D26C, 0xFE53516F, 0xED03A29B, 0x1F682198,
		0x5125DAD3,	0xA34E59D0, 0xB01EAA24, 0x42752927, 0x96BF4DCC, 0x64D4CECF, 0x77843D3B, 0x85EFBE38,
		0xDBFC821C,	0x2997011F, 0x3AC7F2EB, 0xC8AC71E8, 0x1C661503, 0xEE0D9600, 0xFD5D65F4, 0x0F36E6F7,
		0x61C69362,	0x93AD1061, 0x80FDE395, 0x72966096, 0xA65C047D, 0x5437877E, 0x4767748A, 0xB50CF789,
		0xEB1FCBAD,	0x197448AE, 0x0A24BB5A, 0xF84F3859, 0x2C855CB2, 0xDEEEDFB1, 0xCDBE2C45, 0x3FD5AF46,
		0x7198540D,	0x83F3D70E, 0x90A324FA, 0x62C8A7F9, 0xB602C312, 0x44694011, 0x5739B3E5, 0xA55230E6,
		0xFB410CC2,	0x092A8FC1, 0x1A7A7C35, 0xE811FF36, 0x3CDB9BDD, 0xCEB018DE, 0xDDE0EB2A, 0x2F8B6829,
		0x82F63B78,	0x709DB87B, 0x63CD4B8F, 0x91A6C88C, 0x456CAC67, 0xB7072F64, 0xA457DC90, 0x563C5F93,
		0x082F63B7, 0xFA44E0B4, 0xE9141340, 0x1B7F9043, 0xCFB5F4A8, 0x3DDE77AB, 0x2E8E845F, 0xDCE5075C,
		0x92A8FC17, 0x60C37F14, 0x73938CE0, 0x81F80FE3, 0x55326B08, 0xA759E80B, 0xB4091BFF, 0x466298FC,
		0x1871A4D8, 0xEA1A27DB, 0xF94AD42F, 0x0B21572C, 0xDFEB33C7, 0x2D80B0C4, 0x3ED04330, 0xCCBBC033,
		0xA24BB5A6, 0x502036A5, 0x4370C551, 0xB11B4652, 0x65D122B9, 0x97BAA1BA, 0x84EA524E, 0x7681D14D,
		0x2892ED69, 0xDAF96E6A, 0xC9A99D9E, 0x3BC21E9D, 0xEF087A76, 0x1D63F975, 0x0E330A81, 0xFC588982,
		0xB21572C9, 0x407EF1CA, 0x532E023E, 0xA145813D, 0x758FE5D6, 0x87E466D5, 0x94B49521, 0x66DF1622,
		0x38CC2A06, 0xCAA7A905, 0xD9F75AF1, 0x2B9CD9F2, 0xFF56BD19, 0x0D3D3E1A, 0x1E6DCDEE, 0xEC064EED,
		0xC38D26C4, 0x31E6A5C7, 0x22B65633, 0xD0DDD530, 0x0417B1DB, 0xF67C32D8, 0xE52CC12C, 0x1747422F,
		0x49547E0B, 0xBB3FFD08, 0xA86F0EFC, 0x5A048DFF, 0x8ECEE914, 0x7CA56A17, 0x6FF599E3, 0x9D9E1AE0,
		0xD3D3E1AB, 0x21B862A8, 0x32E8915C, 0xC083125F, 0x144976B4, 0xE622F5B7, 0xF5720643, 0x07198540,
		0x590AB964, 0xAB613A67, 0xB831C993, 0x4A5A4A90, 0x9E902E7B, 0x6CFBAD78, 0x7FAB5E8C, 0x8DC0DD8F,
		0xE330A81A, 0x115B2B19, 0x020BD8ED, 0xF0605BEE, 0x24AA3F05, 0xD6C1BC06, 0xC5914FF2, 0x37FACCF1,
		0x69E9F0D5, 0x9B8273D6, 0x88D28022, 0x7AB90321, 0xAE7367CA, 0x5C18E4C9, 0x4F48173D, 0xBD23943E,
		0xF36E6F75, 0x0105EC76, 0x12551F82, 0xE03E9C81, 0x34F4F86A, 0xC69F7B69, 0xD5CF889D, 0x27A40B9E,
		0x79B737BA, 0x8BDCB4B9, 0x988C474D, 0x6AE7C44E, 0xBE2DA0A5, 0x4C4623A6, 0x5F16D052, 0xAD7D5351 };

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
		
		for (uint16_t i=0;i<sizeofin+COUNTERSIZE+CRCSIZE;i++) {
			
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
		
		//uint8_t array[BUFFSIZE];
		uint8_t crcbuff[CRCSIZE];

		//eeprom_read_block(array,(int *) curaddress,sizeofin+CRCSIZE+COUNTERSIZE); //read the data
		
// 		for (int i=0;i<sizeofin+CRCSIZE+COUNTERSIZE;i++) {
// 			printf("%x ",array[i]);
// 		}

		crc_t crccalc;
		crccalc = 0xFFFFFFFF;
		for (uint16_t i=0;i<sizeofin+COUNTERSIZE+CRCSIZE;i++) {

			if (i<sizeofin+COUNTERSIZE) {
				crccalc = pgm_read_dword(&(crc_tab[(crccalc ^ (eeprom_read_byte((uint8_t*)curaddress+i))) & 0xFFL])) ^ (crccalc >> 8);
			} else {
				crcbuff[i-(sizeofin+COUNTERSIZE)]=eeprom_read_byte((uint8_t*)curaddress+i);
			}
		}
		crccalc ^= 0xFFFFFFFF;
		
		//printf("\ncrc 1 %lx\n",crccalc);
		
		//crc_t crccalc=csp_crc32_memory(array,sizeofin+COUNTERSIZE); //calculate the crc of read data
		crc_t crcread=0;

		
		for(uint16_t i=0; i<CRCSIZE;i++) {

			crcread += (crc_t(crcbuff[i])<<i*8); //get the crc read
									
		}
		
		//printf("\ncrc read %lx crc calc %lx\n",crcread,crccalc);
		
		if((crccalc-crcread)!=0) return -2; //compare crc's
		
		// if everything checks out, pass that data out
		for(uint16_t i=0;i<sizeofin;i++) {
			uint8_t * point;

			uint8_t in=eeprom_read_byte((uint8_t*)curaddress+i+COUNTERSIZE);
			*point=in;
			memcpy(data+i, point,1); 
			
		}
		
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
		
		//uint8_t array[BUFFSIZE];

		//memcpy(array+COUNTERSIZE,data,sizeofin+COUNTERSIZE); //copy data to new array, leaving room for counter
				
		uint8_t countbuff[COUNTERSIZE];
		uint8_t crcbuff[CRCSIZE];
		count_t readcount=0;
		
		//printf("start add %u cur add %u",startadd,curaddress);
		
		
		eeprom_read_block(countbuff,(int *) curaddress,COUNTERSIZE); //read current val
		
		for(uint16_t i=0; i<COUNTERSIZE;i++) {
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
		memcpy(countbuff,&readcount,COUNTERSIZE);
		//memcpy(array,&readcount,COUNTERSIZE);

		crc_t crc;
		crc = 0xFFFFFFFF;
		for (uint16_t i=0;i<sizeofin+COUNTERSIZE;i++) {
			uint8_t in=0;
			if (i<COUNTERSIZE) {
				in=countbuff[i];
			} else {
				memcpy(&in,(uint8_t *)(data+i-COUNTERSIZE),1);
			}

			crc = pgm_read_dword(&(crc_tab[(crc ^ in) & 0xFFL])) ^ (crc >> 8);
			//printf("%x ",in);
		}
		crc ^= 0xFFFFFFFF;
		
		//printf("\ncrc 1 %lx\n",crc);

		
		//crc=csp_crc32_memory(array,sizeofin+COUNTERSIZE); //calculate the crc
		
		//printf("\ncrc 2 %lx\n",crc);
		
// 		for (uint16_t i=0;i<CRCSIZE;i++) {
// 			array[sizeofin+COUNTERSIZE+i]=(crc&(0xFFL<<i*8))>>i*8;
// 		}

		for (uint16_t i=0;i<CRCSIZE;i++) {
			crcbuff[i]=(crc&(0xFFL<<i*8))>>i*8;
		}
		

		for(uint16_t i=0;i<sizeofin+COUNTERSIZE+CRCSIZE;i++) {
			if(i<COUNTERSIZE) {
				//counter section
				eeprom_write_byte((uint8_t *)curaddress+i,countbuff[i]);
			} else if(i>=sizeofin+COUNTERSIZE) {
				//crc section
				eeprom_write_byte((uint8_t *)curaddress+i,crcbuff[i-sizeofin-COUNTERSIZE]);
			} else {
				//data section
				uint8_t in=0;
				memcpy(&in,(uint8_t *)(data+i-COUNTERSIZE),1);
				eeprom_write_byte((uint8_t *)curaddress+i,in);
			}
			
		}
		
		//eeprom_read_block(array,(uint8_t *)curaddress,sizeofin+COUNTERSIZE+CRCSIZE);
		
// 		for (int i=0;i<sizeofin+CRCSIZE+COUNTERSIZE;i++) {
//  			printf("%x ",array[i]);
//  		}
 			

		//eeprom_write_block(array,(int *) curaddress,sizeofin+CRCSIZE+COUNTERSIZE);
				
				return 0;

		
	}

	
	uint8_t R_EEPROM::incrementblock(void) {
		if(blockpoint+1>=blocks) return -1;
		blockpoint++;
		printf("block incr\n");
		curaddress=startadd+(sizeofin*blockpoint);
		return 0;
	}

	
	