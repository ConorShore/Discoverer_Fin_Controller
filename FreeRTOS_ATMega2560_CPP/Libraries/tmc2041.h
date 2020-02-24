#include <SPI/spi.h>
#include <fin.h>
#include <stdint.h>

//stepper motor defines, mostly address but some setup stuff

#define	STEPPER_GCONF_ADD 0x00
#define STEPPER_GCONF_DATA(a,b) ((0x006) | (a<<8) | (b<<9)) //a or b =1 will invert stepper dir, 0 wont
#define STEPPER_GSTAT 0x01
#define STEPPER_ISET1_ADD 0x30 // for setting currents to steppers
#define STEPPER_ISET2_ADD 0x50
#define STEPPER_MSCNT1_ADD 0x6A //microstep count
#define STEPPER_MSCNT2_ADD 0x7A
#define STEPPER_MSCURACT1_ADD 0x6B
#define STEPPER_MSCURACT2_ADD 0x7B
#define STEPPER_CHOPCONF1_ADD 0x6C
#define STEPPER_CHOPCONF2_ADD 0x7C

#define STEPPER_DEFAULT_IHOLD 31
#define STEPPER_DEFAULT_IRUN 31

#define STEPPER_1_EEPROM_ADDRESS 0x10
#define STEPPER_2_EEPROM_ADDRESS 0x20

#define STEPPER_SPI_CLK 2000000UL

typedef struct uniman_step_config {
	uint16_t	GCONF;
	uint8_t		IHOLD;
	uint8_t		IRUN;
	uint32_t	CHOPCONF;
}uniman_step_config_t;


typedef struct uniman_step_reg {
	uint8_t status;
	uint8_t address;
	uint8_t data[4];

}uniman_step_reg_t;

typedef struct uniman_step_reg_32 {
	uint8_t status;
	uint8_t address;
	uint32_t data;

}uniman_step_reg_32_t;



class tmc2041 {
	
	private:
	uniman_step_config_t config;
	void (*init_cs_en)();
	void (*cson)();
	void (*csoff)();
	void (*enstep)();
	void (*disstep)();
	void startSPI(void);
	void endSPI(void);
	uint8_t eeprom_address;

	
	
	
	public:
	tmc2041(void (csinitin()),void (csonin()),
	void (csoffin()),void (enstepin()), void (disstepin()),
	uniman_step_config_t configin,uint8_t eeprom_addressin);
	gs_fin_cmd_error_t set_speed(uint8_t);
	gs_fin_cmd_error_t set_pos1(uint16_t pos, uint8_t speed);
	gs_fin_cmd_error_t set_pos2(uint16_t pos, uint8_t speed);
	void writereg(uniman_step_reg_t * databack,uint8_t amount); // for writing individual bytes
	void writereg(uniman_step_reg_32_t * databack); //for writing the full 32bit
	void readreg(uniman_step_reg_t * databack); // for reading individual bytes
	void readreg(uniman_step_reg_32_t * databack); // for reading the full 32bit
	gs_fin_cmd_error_t saveconfig(void); //saves current config to eeprom
	gs_fin_cmd_error_t updateconfig(uniman_step_config_t * confin);
	
	
};
