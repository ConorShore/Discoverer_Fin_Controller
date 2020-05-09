#include <SPI/spi.h>
#include <fin.h>
#include <stdint.h>


//stepper motor defines, mostly address but some setup stuff

#define	STEPPER_GCONF_ADD 0x00
#define STEPPER_GCONF_DEFAULT 0x006
#define STEPPER_GCONF_DATA(a,b) ((STEPPER_GCONF_DEFAULT) | (a<<8) | (b<<9)) //a or b =1 will invert stepper dir, 0 wont
#define STEPPER_GSTAT 0x01
#define STEPPER_ISET1_ADD 0x30 // for setting currents to steppers
#define STEPPER_ISET2_ADD 0x50
#define STEPPER_MSCNT1_ADD 0x6A //microstep count
#define STEPPER_MSCNT2_ADD 0x7A
#define STEPPER_MSCURACT1_ADD 0x6B
#define STEPPER_MSCURACT2_ADD 0x7B
#define STEPPER_CHOPCONF1_ADD 0x6C
#define STEPPER_CHOPCONF2_ADD 0x7C
#define STEPPER_CHOPCONF_DEFAULT 0x000300C3//0x080100C3

#define STEPPER_FSTEP_DELAY (uint16_t(2048))

#define STEPPER_DEFAULT_IHOLD 1
#define STEPPER_DEFAULT_IRUN 15
#define STEPPER_DEFAULT_IHOLDDELAY 1

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
	gs_fin_config_t runconf;
	void (*init_pins)();
	void startSPI(void);
	void endSPI(void);
	uint8_t eeprom_address;
	void (*cson)();
	void (*csoff)();


	static uint8_t speed;
	
	
	public:
	tmc2041(void (csinitin()),void (csonin()),
	void (csoffin()),void (enstepin()), void (disstepin()), void(stepfuncin(uint8_t a)),void(dirfuncin(uint8_t a,uint8_t t)),
	uniman_step_config_t configin,uint8_t eeprom_addressin);

	void writereg(uniman_step_reg_t * databack,uint8_t amount); // for writing individual bytes
	void writereg(uniman_step_reg_32_t * databack); //for writing the full 32bit
	void readreg(uniman_step_reg_t * databack); // for reading individual bytes
	void readreg(uniman_step_reg_32_t * databack); // for reading the full 32bit
	gs_fin_cmd_error_t saveconfig(void); //saves current config to eeprom
	gs_fin_cmd_error_t updateconfig(uniman_step_config_t * confin,gs_fin_config_t * confin2);

	void (*enstep)();
	void (*disstep)();
	
	void step(uint8_t a,uint8_t dir);
	
	void (*stepfunc)(uint8_t a);
	void (*dirfunc)(uint8_t a, uint8_t t);

	
};

