#include "expander-i2c.h"
#include "bcm2835.h"
#include "ADE9078registers.h"


#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

typedef struct {
  // All gains are 2 bits. Options: 1, 2, 3, 4
  uint8_t vAGain;
  uint8_t vBGain;
  uint8_t vCGain;

  uint8_t iAGain;
  uint8_t iBGain;
  uint8_t iCGain;
  uint8_t iNGain;

  uint32_t powerAGain;
  uint32_t powerBGain;
  uint32_t powerCGain;

  uint8_t vConsel;
  uint8_t iConsel;
}InitializationSettings;

#define VERSION_16 0x4FE //Reset: 0x0040 Access: R

#define ADE9078_VERBOSE_DEBUG


static void pabort(const char *s)
{
        perror(s);
        abort();
}

const uint8_t WRITE = 0b00000000; //This value tells the ADE9078 that data is to be written to the requested register.
const uint8_t READ = 0b10000000;  //This value tells the ADE9078 that data is to be read from the requested register.


//////////
// Init SPIdev
//////////
 
//////////
// Read n bytes from the 2 bytes add1 add2 address
//////////

uint8_t functionBitVal(uint16_t addr, uint8_t byteVal)
{
//Returns as integer an address of a specified byte - basically a byte controlled shift register with "byteVal" controlling the byte that is read and returned
  uint16_t x = ((addr >> (8*byteVal)) & 0xff);

  #ifdef ADE9078_VERBOSE_DEBUG
   printf(" functionBitVal function (separates high and low command bytes of provided addresses) details: ");
   printf(" Address input (dec): ");
   printf("%d\n",addr);
   printf(" Byte requested (dec): ");
   printf("%d\n", byteVal);
   printf(" Returned Value (dec): ");
   printf("%d\n", x);
   printf(" Returned Value (HEX): ");
   printf("%02x\n", x);
   printf(" functionBitVal function completed\n\n");
  #endif

  return x;
}

uint16_t ADE9078_spiRead16(uint16_t address, expander_t *exp) { //This is the algorithm that reads from a register in the ADE9078. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
    #ifdef ADE9078_VERBOSE_DEBUG
     printf(" spiRead16 function started \n");
    #endif
   //Prepare the 12 bit command header from the inbound address provided to the function
    expander_setAllPinsGPIO(exp);
   uint16_t temp_address, readval_unsigned;
   temp_address = (((address << 4) & 0xFFF0)+8); //shift address  to align with cmd packet, convert the 16 bit address into the 12 bit command header. + 8 for isRead versus write
   uint8_t commandHeader1 = functionBitVal(temp_address, 1); //lookup and return first byte (MSB) of the 12 bit command header, sent first
   uint8_t commandHeader2 = functionBitVal(temp_address, 0); //lookup and return second byte (LSB) of the 12 bit command header, sent second

    uint8_t one, two; //holders for the read values from the SPI Transfer


    expander_printGPIO(exp);

    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      exit(EXIT_FAILURE);
    }

    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      exit(EXIT_FAILURE);
    }
      expander_resetOnlyPinSetOthersGPIO(exp, 5);
      bcm2835_spi_transfer(commandHeader1); //Send MSB
      bcm2835_spi_transfer(commandHeader2); //Send MSB
      one = bcm2835_spi_transfer(WRITE);  //dummy write MSB, read out MSB
      two = bcm2835_spi_transfer(WRITE);  //dummy write LSB, read out LSB
      expander_setPinGPIO(exp,5);
      bcm2835_spi_end();

	#ifdef AVRESP8266 //Arduino SPI Routine
    // beginTransaction is first
    SPI.beginTransaction(defaultSPISettings);  // Clock is high when inactive. Read at rising edge: SPIMODE3.
    digitalWrite(_SS, LOW);  //Enable data transfer by bringing SS line LOW
    SPI.transfer(commandHeader1);  //Transfer first byte (MSB), command
    SPI.transfer(commandHeader2);  ;//Transfer second byte (LSB), command
    //Read in values sequentially and bitshift for a 32 bit entry
    one = SPI.transfer(dummyWrite);  //MSB Byte 1  (Read in data on dummy write (null MOSI signal))
    two = SPI.transfer(dummyWrite);  //LSB Byte 2  (Read in data on dummy write (null MOSI signal))
    digitalWrite(_SS, HIGH);  //End data transfer by bringing SS line HIGH
    SPI.endTransaction();      // end SPI Transaction
	#endif

    #ifdef ADE9078_VERBOSE_DEBUG
     printf(" spiRead16 function details: \n");
     printf(" Command Header: \n");
     printf("%02x\n",commandHeader1);
     printf("%02x\n",commandHeader2);
     printf(" Address Byte 1(MSB)[HEX]: \n");
     printf(" Returned bytes (1(MSB) and 2) [HEX]: \n");
     printf("%02x", one); //print MSB
     printf("\n");
     printf("%02x\n", two);  //print LSB
     printf(" spiRead16 function completed \n");
    #endif

	  readval_unsigned = (one << 8);  //Process MSB  (Alternate bitshift algorithm)
    readval_unsigned = readval_unsigned + two;  //Process LSB
	return readval_unsigned;
}

uint32_t ADE9078_spiRead32(uint16_t address,expander_t *exp) { //This is the algorithm that reads from a 32 bit register in the ADE9078. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.  Caution, some register elements contain information that is only 24 bit with padding on the MSB
  #ifdef ADE9078_VERBOSE_DEBUG
   printf(" spiRead32 function started \n");
  #endif

   //Prepare the 12 bit command header from the inbound address provided to the function
   uint16_t temp_address;
   temp_address = (((address << 4) & 0xFFF0)+8); //shift address  to align with cmd packet, convert the 16 bit address into the 12 bit command header. + 8 for isRead versus write
   uint8_t commandHeader1 = functionBitVal(temp_address, 1); //lookup and return first byte (MSB) of the 12 bit command header, sent first
   uint8_t commandHeader2 = functionBitVal(temp_address, 0); //lookup and return second byte (LSB) of the 12 bit command header, sent second

  uint8_t one, two, three, four; //holders for the read values from the SPI Transfer


    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      exit(EXIT_FAILURE);
    }

    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      exit(EXIT_FAILURE);
    }
    expander_resetOnlyPinSetOthersGPIO(exp, 5);
    expander_printGPIO(exp);
    bcm2835_spi_transfer(commandHeader1); //Send MSB
    bcm2835_spi_transfer(commandHeader2); //Send MSB
    one = bcm2835_spi_transfer(WRITE);  //dummy write MSB, read out MSB
    two = bcm2835_spi_transfer(WRITE);  //dummy write LSB, read out LSB
    three = bcm2835_spi_transfer(WRITE);  //dummy write LSB, read out LSB
    four = bcm2835_spi_transfer(WRITE);  //dummy write LSB, read out LSB
    expander_setPinGPIO(exp,5);
    bcm2835_spi_end();


  #ifdef AVRESP8266 //Arduino SPI Routine
  SPI.beginTransaction(defaultSPISettings);  // Clock is high when inactive. Read at rising edge: SPIMODE3.
  digitalWrite(_SS, LOW);  //Enable data transfer by bringing SS line LOW
  SPI.transfer(commandHeader1);  //MSB Byte 1
  SPI.transfer(commandHeader2);
  //Read in values sequentially and bitshift for a 32 bit entry
  one = SPI.transfer(dummyWrite); //MSB Byte 1  (Read in data on dummy write (null MOSI signal))
  two = SPI.transfer(dummyWrite);   // (Read in data on dummy write (null MOSI signal))
  three = SPI.transfer(dummyWrite);   // (Read in data on dummy write (null MOSI signal))
  four = SPI.transfer(dummyWrite); //LSB Byte 4  (Read in data on dummy write (null MOSI signal))
  digitalWrite(_SS, HIGH);  //End data transfer by bringing SS line HIGH
  SPI.endTransaction();
  #endif

  #ifdef ADE9078_VERBOSE_DEBUG
   printf(" Returned bytes 1-4, 1 is MSB [HEX]: \n");
   printf(" spiRead32 function details: \n");
   printf(" Command Header: ");
   printf("%02x", commandHeader1);
   printf("%02x", commandHeader2);
   printf(" Returned bytes (1(MSB) to 4)[BINARY]: ");
   printf("%02x", one);
   printf(" \n");
   printf("%02x", two);
   printf(" \n");
   printf("%02x", three);
   printf(" \n");
   printf("%02x\n", four);
  #endif

  //Post-read packing and bitshifting operations
  return (((uint32_t) one << 24) + ((uint32_t) two << 16) + ((uint32_t) three << 8) + (uint32_t) four);
}


void ADE9078_spiWrite32(uint16_t address, uint32_t data,expander_t *exp) {

	//Prepare the 12 bit command header from the inbound address provided to the function
	uint16_t temp_address;
	temp_address = ((address << 4) & 0xFFF0);	//shift address  to align with cmd packet, convert the 16 bit address into the 12 bit command header
	uint8_t commandHeader1 = functionBitVal(temp_address, 1); //lookup and return first byte (MSB) of the 12 bit command header, sent first
	uint8_t commandHeader2 = functionBitVal(temp_address, 0); //lookup and return second byte (LSB) of the 12 bit command header, sent second

	//Structure inbound function data to send out over SPI byte by byte with MSB first - 	//Perform bitshifts to structure the values: To understand these shifts, picture this group of 1's being modified - Below is a 32 bit int. We're grabbing 1 byte out at a time. byteFour is the left most byte// 1111 1111 1111 1111 1111 1111 1111 1111
	uint8_t byteFour = (data >> 24);
  uint8_t byteThree = (data & 0xFFFFFF) >> 16;
  uint8_t byteTwo = (data & 0xFFFF) >> 8;
  uint8_t byteOne = (data & 0xFF);


    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      exit(EXIT_FAILURE);
    }

    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      exit(EXIT_FAILURE);
    }
    expander_resetOnlyPinSetOthersGPIO(exp, 5);
    expander_printGPIO(exp);
    bcm2835_spi_transfer(commandHeader1); //Send MSB
    bcm2835_spi_transfer(commandHeader2); //Send MSB
    bcm2835_spi_transfer(byteFour);  //dummy write MSB, read out MSB
    bcm2835_spi_transfer(byteThree);  //dummy write LSB, read out LSB
    bcm2835_spi_transfer(byteTwo);  //dummy write LSB, read out LSB
    bcm2835_spi_transfer(byteOne);  //dummy write LSB, read out LSB
    expander_setPinGPIO(exp,5);
    bcm2835_spi_end();
	

    #ifdef ADE9078_VERBOSE_DEBUG
    //  printf(" ADE9078::spiRead32 function details: \n");
    //  printf("Command Header: " + commandHeader1 + commandHeader2);
    //  printf(" Wrote bytes (4(MSB) to 1)[BINARY]: \n");
    //  printf(byteFour, BIN);
    //  printf(" \n");
    //  printf(byteThree, BIN);
    //  printf(" \n");
    //  printf(byteTwo, BIN);
    //  printf(" \n");
    //  printf(byteOne, BIN);
    //  printf(" ADE9078::spiRead32 function completed \n");
    #endif

  }

void ADE9078_spiWrite16(uint16_t address, uint16_t data,expander_t *exp) {

   //Prepare the 12 bit command header from the inbound address provided to the function
   uint16_t temp_address;
   temp_address = ((address << 4) & 0xFFF0);	//shift address to align with cmd packet, convert the 16 bit address into the 12 bit command header
   uint8_t commandHeader1 = functionBitVal(temp_address, 1); //lookup and return first byte (MSB) of the 12 bit command header, sent first
   uint8_t commandHeader2 = functionBitVal(temp_address, 0); //lookup and return second byte (LSB) of the 12 bit command header, sent second
  
  uint8_t byteTwo = (data >> 8);
  uint8_t byteOne = (data & 0xFF);
  //Structure inbound function data into two bytes to send out over SPI sequentially, MSB is sent first

if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      exit(EXIT_FAILURE);
    }

    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      exit(EXIT_FAILURE);
    }
    expander_resetOnlyPinSetOthersGPIO(exp, 5);
    expander_printGPIO(exp);
    bcm2835_spi_transfer(commandHeader1); //Send MSB
    bcm2835_spi_transfer(commandHeader2); //Send MSB
    bcm2835_spi_transfer(byteTwo);  //dummy write LSB, read out LSB
    bcm2835_spi_transfer(byteOne);  //dummy write LSB, read out LSB
    expander_setPinGPIO(exp,5);
    bcm2835_spi_end();

  #ifdef ADE9078_VERBOSE_DEBUG
  //  printf(" ADE9078::spiRead32 function details: \n");
  //  printf("Command Header: \n");
  //  printf("%d \n",commandHeader1);
  //  printf("%d \n",commandHeader2);
  //  printf(" Wrote bytes (2(MSB) to 1)[BINARY]: \n");
  //  printf("0x %02x \n",byteTwo);
  //  printf(" \n");
  //  printf("0x %02x \n",byteOne);
  //  printf(" ADE9078::spiRead32 function completed \n");
  #endif
}

  uint16_t ADE9078_getVersion(expander_t *exp){

	  return ADE9078_spiRead16(VERSION_16, exp);
}

void ADE9078_initialize(expander_t *exp){

  #ifdef ADE9078_VERBOSE_DEBUG
   printf(" ADE9078:initialize function started\n"); //wiring configuration defined in VCONSEL and ICONSEL registers init. in this function
  #endif
  InitializationSettings is = { 0,};
    /* //For reference, the following registers are written to on bootup for the ADE9000 (from ADE9000 Arduino Library Sample Code, note these ICs are similar but do not have identical functionality!)
   SPI_Write_16(ADDR_PGA_GAIN,ADE9000_PGA_GAIN);
 	 SPI_Write_32(ADDR_CONFIG0,ADE9000_CONFIG0);
	 SPI_Write_16(ADDR_CONFIG1,ADE9000_CONFIG1);
	 SPI_Write_16(ADDR_CONFIG2,ADE9000_CONFIG2);
	 SPI_Write_16(ADDR_CONFIG3,ADE9000_CONFIG3);
	 SPI_Write_16(ADDR_ACCMODE,ADE9000_ACCMODE);
	 SPI_Write_16(ADDR_TEMP_CFG,ADE9000_TEMP_CFG);
	 SPI_Write_16(ADDR_ZX_LP_SEL,ADE9000_ZX_LP_SEL);
	 SPI_Write_32(ADDR_MASK0,ADE9000_MASK0);
	 SPI_Write_32(ADDR_MASK1,ADE9000_MASK1);
	 SPI_Write_32(ADDR_EVENT_MASK,ADE9000_EVENT_MASK);
	 SPI_Write_16(ADDR_WFB_CFG,ADE9000_WFB_CFG);
	 SPI_Write_32(ADDR_VLEVEL,ADE9000_VLEVEL);
	 SPI_Write_32(ADDR_DICOEFF,ADE9000_DICOEFF);
	 SPI_Write_16(ADDR_EGY_TIME,ADE9000_EGY_TIME);
	 SPI_Write_16(ADDR_EP_CFG,ADE9000_EP_CFG);		//Energy accumulation ON
	 SPI_Write_16(ADDR_RUN,ADE9000_RUN_ON);		//DSP ON
	 */


  // ESP32 Architecture setup
  #ifdef ESP32ARCH  //example SPI routine for the ESP32
  spy = spiStartBus(VSPI, SPI_CLOCK_DIV16, SPI_MODE3, SPI_MSBFIRST);
  spiAttachSCK(spy, -1);
  spiAttachMOSI(spy, -1);
  spiAttachMISO(spy, -1);
  pinMode(_SS, OUTPUT);
  spiStopBus(spy);
  digitalWrite(_SS, HIGH); //Disable data transfer by bringing SS line HIGH
  delay(50);
  #endif

  #ifdef AVRESP8266 //Arduino SPI Routine
  SPI.begin();
  SPI.beginTransaction(defaultSPISettings);  // Clock is high when inactive. Read at rising edge: SPIMODE3 or 0 modes.
  pinMode(_SS, OUTPUT); // FYI: SS is pin 10 by Arduino's SPI library on many boards (including the UNO), set SS pin as Output
  SPI.setBitOrder(MSBFIRST);  //Define MSB as first (explicitly)
  SPI.endTransaction(); //end SPI communication
  digitalWrite(_SS, HIGH); //Initialize pin as HIGH to bring communication inactive
  delay(50);
  #endif

  // Page 56 of datasheet quick start
  // #1: Ensure power sequence completed
  delay(30);

  // Is always printing right now. Might be an issue?
  // if (!checkBit((int)read32BitAndScale(STATUS1_32), 16)) {
  //   printf("WARNING, POWER UP MAY NOT BE FINISHED\n");
  // }
   // #2: Configure Gains
   ADE9078_spiWrite32(APGAIN_32, is.powerAGain, exp);
   ADE9078_spiWrite32(BPGAIN_32, is.powerBGain, exp);
   ADE9078_spiWrite32(CPGAIN_32, is.powerCGain, exp);

   uint16_t pgaGain = (is.vCGain << 12) + (is.vBGain << 10) + (is.vCGain << 8) +   // first 2 reserved, next 6 are v gains, next 8 are i gains.
                      (is.iNGain << 6) + (is.iCGain << 4) + (is.iBGain << 2) + is.iAGain;
   ADE9078_spiWrite16(PGA_GAIN_16, pgaGain, exp);
   uint32_t vLevelData = 0x117514;  // #5 : Write VLevel 0x117514
   ADE9078_spiWrite32(VLEVEL_32, vLevelData, exp); // #5

  ADE9078_spiWrite16(CONFIG0_32, 0x00000000, exp);  // #7:  If current transformers are used, INTEN and ININTEN in the CONFIG0 register must = 0
  // Table 24 to determine how to configure ICONSEL and VCONSEL in the ACCMODE register
  uint16_t settingsACCMODE = (is.iConsel << 6) + (is.vConsel << 5);

	ADE9078_spiWrite16(ACCMODE_16, settingsACCMODE, exp); // chooses the wiring mode (delta/Wye, Blondel vs. Non-blondel) to push up in initial config, Need the other if statements for all configuration modes

  ADE9078_spiWrite16(RUN_16, 1, exp);  // 8: Write 1 to Run register
  ADE9078_spiWrite16(EP_CFG_16, 1, exp);  // 9: Write 1 to EP_CFG register

  /*
  Potentially useful registers to configure:
  The following were in the 9078:
    0x49A ZX_LP_SEL : to configure "zero crossing signal"
    0x41F PHNOLOAD : To say if something is "no load".
    Phase calibrations, such as APHCAL1_32
  */
  ADE9078_spiWrite16(CONFIG1_16, 0x0000, exp);
  ADE9078_spiWrite16(CONFIG2_16, 0x0000, exp);
  ADE9078_spiWrite16(CONFIG3_16, 0x0000, exp);
  ADE9078_spiWrite32(DICOEFF_32, 0xFFFFE000, exp); // Recommended by datasheet

  /* Registers configured in ADE9000 code */
  // zx_lp_sel
  // mask0, mask1, event_mask,
  // wfb_cfg,
  ADE9078_spiWrite16(EGY_TIME_16, 0x0001, exp);
  ADE9078_spiWrite16(EP_CFG_16, 0x0021, exp); // RD_EST_EN=1, EGY_LD_ACCUM=0, EGY_TMR_MODE=0, EGY_PWR_EN=1

  #ifdef ADE9078_VERBOSE_DEBUG
  //  printf(" ADE9078:initialize function completed. Showing values and registers written \n");
  //  printf(" APGAIN: ");
  //  printf("%d \n",is->powerAGain);
  //  printf(" BPGAIN: ");
  //  printf("%d \n",is->powerBGain);
  //  printf(" CPGAIN: ");
  //  printf("%d \n",is->powerCGain);
  //  printf(" PGA_GAIN: ");
  //  printf("%d \n",is->pgaGain);
  //  printf(" VLEVEL: ");
  //  printf("%d \n",is->vLevelData);
  //  printf(" CONFIG0-3, ALL 0'Sn \n");
  //  printf(" ACCMODE: ");
  //  printf("%d \n",is->settingsACCMODE);
  //  printf(" RUN: \n");
  //  printf(1);
  //  printf(" EP_CFG: \n");
  //  printf(1);
  //  printf(" DICOEFF: \n");
  //  printf("%80x"0xFFFFE000);
  #endif
}





uint32_t ADE9078_getInstVoltageA(expander_t *exp){
	uint32_t value=0;
	value=ADE9078_spiRead32(AV_PCF_32, exp);
return value;
}



int main(){

    expander_t *exp1 = expander_init(0x26);
    expander_t *exp2 = expander_init(0x27);

    ADE9078_initialize(exp2);
    //spi_init();
    // uint8_t send_data = 0x23;
    // uint8_t read_data = bcm2835_spi_transfer(send_data);
    // printf("Sent to SPI: 0x%02X. Read back from SPI: 0x%02X.\n", send_data, read_data);
    // if (send_data != read_data)
    //   printf("Do you have the loopback from MOSI to MISO connected?\n");
    // bcm2835_spi_end();

    expander_resetAllPinsGPIO(exp1);
    expander_setPinGPIO(exp1, 0);
    sleep(2);
    expander_printGPIO(exp1);

    printf("version %04x\n",ADE9078_getVersion(exp2));
    printf("tension %dV\n",ADE9078_getInstVoltageA(exp2));

    expander_resetPinGPIO(exp1, 0);

    bcm2835_close();

  return 0;
}