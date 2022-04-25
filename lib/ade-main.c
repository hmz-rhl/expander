#include "expander-i2c.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define VERSION_16 0x4FE //Reset: 0x0040 Access: R

#define ADE9078_VERBOSE_DEBUG

int spi_cs0_fd;				//file descriptor for the SPI device
int spi_cs1_fd;				//file descriptor for the SPI device
unsigned char spi_mode;
unsigned char spi_bitsPerWord;
unsigned int spi_speed;


//////////
// Init SPIdev
//////////
int SpiOpenPort (int spi_device)
{
	int status_value = -1;
    int *spi_cs_fd;


    //----- SET SPI MODE -----
    //SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
    //SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
    //SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
    //SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
    spi_mode = SPI_MODE_0;
    
    //----- SET BITS PER WORD -----
    spi_bitsPerWord = 8;
    
    //----- SET SPI BUS SPEED -----
    spi_speed = 1000000;		//1000000 = 1MHz (1uS per bit) 


    if (spi_device)
    	spi_cs_fd = &spi_cs1_fd;
    else
    	spi_cs_fd = &spi_cs0_fd;


    if (spi_device)
    	*spi_cs_fd = open("/dev/spidev0.1", O_RDWR);
    else
    	*spi_cs_fd = open("/dev/spidev0.0", O_RDWR);

    if (*spi_cs_fd < 0)
    {
        perror("Error - Could not open SPI device\n");
        exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_MODE, &spi_mode);
    if(status_value < 0)
    {
        perror("Could not set SPIMode (WR)...ioctl fail\n");
        exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_MODE, &spi_mode);
    if(status_value < 0)
    {
      perror("Could not set SPIMode (RD)...ioctl fail\n");
      exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
    if(status_value < 0)
    {
      perror("Could not set SPI bitsPerWord (WR)...ioctl fail\n");
      exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
    if(status_value < 0)
    {
      perror("Could not set SPI bitsPerWord(RD)...ioctl fail\n");
      exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if(status_value < 0)
    {
      perror("Could not set SPI speed (WR)...ioctl fail\n");
      exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if(status_value < 0)
    {
      perror("Could not set SPI speed (RD)...ioctl fail\n");
      exit(1);
    }
    return(status_value);
}

 
//////////
// Read n bytes from the 2 bytes add1 add2 address
//////////
 
int SpiWriteAndRead (int SpiDevice, unsigned char *TxData, unsigned char *RxData, int Length, int LeaveCsLow)
{
	struct spi_ioc_transfer spi;
	int i = 0;
	int retVal = -1;
	int spi_cs_fd;

	if (SpiDevice)
		spi_cs_fd = spi_cs1_fd;
	else
		spi_cs_fd = spi_cs0_fd;

	spi.tx_buf = (unsigned long)TxData;		//transmit from "data"
	spi.rx_buf = (unsigned long)RxData;		//receive into "data"
	spi.len = Length;
	spi.delay_usecs = 0;
	spi.speed_hz = spi_speed;
	spi.bits_per_word = spi_bitsPerWord;
	spi.cs_change = LeaveCsLow;						//0=Set CS high after a transfer, 1=leave CS set low

	retVal =ioctl (spi_cs_fd, SPI_IOC_MESSAGE(1), &spi);
  

	if(retVal < 0)
	{
		perror("Error - Problem transmitting spi data..ioctl\n");
		exit(1);
	}

	return retVal;
}

int SpiClosePort (int fd)
{
	int status_value = -1;
    int *spi_cs_fd;

    if (fd)
    	spi_cs_fd = &spi_cs1_fd;
    else
    	spi_cs_fd = &spi_cs0_fd;


    status_value = close(*spi_cs_fd);
    if(status_value < 0)
    {
    	perror("Error - Could not close SPI device\n");
    	exit(1);
    }
    return(status_value);
}

{
// void initializeADE9078(int fd,expander_t *exp){

//   #ifdef ADE9078_VERBOSE_DEBUG
//    printf(" ADE9078:initialize function started\n"); //wiring configuration defined in VCONSEL and ICONSEL registers init. in this function
//   #endif

//     /* //For reference, the following registers are written to on bootup for the ADE9000 (from ADE9000 Arduino Library Sample Code, note these ICs are similar but do not have identical functionality!)
//    SPI_Write_16(ADDR_PGA_GAIN,ADE9000_PGA_GAIN);
//  	 SPI_Write_32(ADDR_CONFIG0,ADE9000_CONFIG0);
// 	 SPI_Write_16(ADDR_CONFIG1,ADE9000_CONFIG1);
// 	 SPI_Write_16(ADDR_CONFIG2,ADE9000_CONFIG2);
// 	 SPI_Write_16(ADDR_CONFIG3,ADE9000_CONFIG3);
// 	 SPI_Write_16(ADDR_ACCMODE,ADE9000_ACCMODE);
// 	 SPI_Write_16(ADDR_TEMP_CFG,ADE9000_TEMP_CFG);
// 	 SPI_Write_16(ADDR_ZX_LP_SEL,ADE9000_ZX_LP_SEL);
// 	 SPI_Write_32(ADDR_MASK0,ADE9000_MASK0);
// 	 SPI_Write_32(ADDR_MASK1,ADE9000_MASK1);
// 	 SPI_Write_32(ADDR_EVENT_MASK,ADE9000_EVENT_MASK);
// 	 SPI_Write_16(ADDR_WFB_CFG,ADE9000_WFB_CFG);
// 	 SPI_Write_32(ADDR_VLEVEL,ADE9000_VLEVEL);
// 	 SPI_Write_32(ADDR_DICOEFF,ADE9000_DICOEFF);
// 	 SPI_Write_16(ADDR_EGY_TIME,ADE9000_EGY_TIME);
// 	 SPI_Write_16(ADDR_EP_CFG,ADE9000_EP_CFG);		//Energy accumulation ON
// 	 SPI_Write_16(ADDR_RUN,ADE9000_RUN_ON);		//DSP ON
// 	 */

// // rendre la communication inactive
//     expander_setAllGPIO(exp);

//   // ESP32 Architecture setup
//   #ifdef ESP32ARCH  //example SPI routine for the ESP32
//   spy = spiStartBus(VSPI, SPI_CLOCK_DIV16, SPI_MODE3, SPI_MSBFIRST);
//   spiAttachSCK(spy, -1);
//   spiAttachMOSI(spy, -1);
//   spiAttachMISO(spy, -1);
//   pinMode(_SS, OUTPUT);
//   spiStopBus(spy);
//   digitalWrite(_SS, HIGH); //Disable data transfer by bringing SS line HIGH
//   delay(50);
//   #endif

//   #ifdef AVRESP8266 //Arduino SPI Routine
//   SPI.begin();
//   SPI.beginTransaction(defaultSPISettings);  // Clock is high when inactive. Read at rising edge: SPIMODE3 or 0 modes.
//   pinMode(_SS, OUTPUT); // FYI: SS is pin 10 by Arduino's SPI library on many boards (including the UNO), set SS pin as Output
//   SPI.setBitOrder(MSBFIRST);  //Define MSB as first (explicitly)
//   SPI.endTransaction(); //end SPI communication
//   digitalWrite(_SS, HIGH); //Initialize pin as HIGH to bring communication inactive
//   delay(50);
//   #endif

//   // Page 56 of datasheet quick start
//   // #1: Ensure power sequence completed
//   delay(30);

//   // Is always printing right now. Might be an issue?
//   // if (!checkBit((int)read32BitAndScale(STATUS1_32), 16)) {
//   //   printf("WARNING, POWER UP MAY NOT BE FINISHED\n");
//   // }
//    // #2: Configure Gains
//    spiWrite32(APGAIN_32, is->powerAGain);
//    spiWrite32(BPGAIN_32, is->powerBGain);
//    spiWrite32(CPGAIN_32, is->powerCGain);

//    uint16_t pgaGain = (is->vCGain << 12) + (is->vBGain << 10) + (is->vCGain << 8) +   // first 2 reserved, next 6 are v gains, next 8 are i gains.
//                       (is->iNGain << 6) + (is->iCGain << 4) + (is->iBGain << 2) + is->iAGain;
//    spiWrite16(PGA_GAIN_16, pgaGain);
//    uint32_t vLevelData = 0x117514;  // #5 : Write VLevel 0x117514
//    spiWrite32(VLEVEL_32, vLevelData); // #5

//   spiWrite16(CONFIG0_32, 0x00000000);  // #7:  If current transformers are used, INTEN and ININTEN in the CONFIG0 register must = 0
//   // Table 24 to determine how to configure ICONSEL and VCONSEL in the ACCMODE register
//   uint16_t settingsACCMODE = (is->iConsel << 6) + (is->vConsel << 5);

// 	spiWrite16(ACCMODE_16, settingsACCMODE); // chooses the wiring mode (delta/Wye, Blondel vs. Non-blondel) to push up in initial config, Need the other if statements for all configuration modes

//   spiWrite16(RUN_16, 1);  // 8: Write 1 to Run register
//   spiWrite16(EP_CFG_16, 1);  // 9: Write 1 to EP_CFG register

//   /*
//   Potentially useful registers to configure:
//   The following were in the 9078:
//     0x49A ZX_LP_SEL : to configure "zero crossing signal"
//     0x41F PHNOLOAD : To say if something is "no load".
//     Phase calibrations, such as APHCAL1_32
//   */
//   spiWrite16(CONFIG1_16, 0x0000);
//   spiWrite16(CONFIG2_16, 0x0000);
//   spiWrite16(CONFIG3_16, 0x0000);
//   spiWrite32(DICOEFF_32, 0xFFFFE000); // Recommended by datasheet

//   /* Registers configured in ADE9000 code */
//   // zx_lp_sel
//   // mask0, mask1, event_mask,
//   // wfb_cfg,
//   spiWrite16(EGY_TIME_16, 0x0001);
//   spiWrite16(EP_CFG_16, 0x0021); // RD_EST_EN=1, EGY_LD_ACCUM=0, EGY_TMR_MODE=0, EGY_PWR_EN=1

//   #ifdef ADE9078_VERBOSE_DEBUG
//    printf(" ADE9078:initialize function completed. Showing values and registers written \n");
//    printf(" APGAIN: \n");
//    printf(is->powerAGain);
//    printf(" BPGAIN: \n");
//    printf(is->powerBGain);
//    printf(" CPGAIN: \n");
//    printf(is->powerCGain);
//    printf(" PGA_GAIN: \n");
//    printf(pgaGain);
//    printf(" VLEVEL: \n");
//    printf(vLevelData);
//    printf(" CONFIG0-3, ALL 0'S\n");
//    printf(" ACCMODE: \n");
//    printf(settingsACCMODE);
//    printf(" RUN: \n");
//    printf(1);
//    printf(" EP_CFG: \n");
//    printf(1);
//    printf(" DICOEFF: \n");
//    printf(0xFFFFE000);
//   #endif
// }
}

uint8_t ADE9078_getVersion(){
	return ADE9078_spiRead16(VERSION_16);
}

uint16_t ADE9078_spiRead16(uint16_t address, expander_t *exp, int fd) { //This is the algorithm that reads from a register in the ADE9078. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
    #ifdef ADE9078_VERBOSE_DEBUG
     printf(" ADE9078::spiRead16 function started \n");
    #endif
   //Prepare the 12 bit command header from the inbound address provided to the function
    expander_setAllPinsGPIO(exp);
   uint16_t temp_address, readval_unsigned;
   temp_address = (((address << 4) & 0xFFF0)+8); //shift address  to align with cmd packet, convert the 16 bit address into the 12 bit command header. + 8 for isRead versus write
   uint8_t commandHeader1 = functionBitVal(temp_address, 1); //lookup and return first byte (MSB) of the 12 bit command header, sent first
   uint8_t commandHeader2 = functionBitVal(temp_address, 0); //lookup and return second byte (LSB) of the 12 bit command header, sent second

    uint8_t one, two; //holders for the read values from the SPI Transfer

  uint8_t tx_data = {commandHeader1, commandHeader2, WRITE, WRITE};
  uint8_t rx_data[128];

    expander_resetOnlyPinSetOthersGPIO(exp, 5);
    SpiWriteAndRead(0,tx_data, rx_data,16,1);
  #ifdef RASPBERRYPIZ //Arduino SPI Routine

    int status = SpiOpenPort(0);
    if(status < 0) exit_failure();


    SpiWriteAndRead(0,tx_data,rx_data,128,0);

    one = rx_data[0];  //MSB Byte 1  (Read in data on dummy write (null MOSI signal)) - only one needed as 1 byte
    two = rx_data[1];  //"LSB "Byte 2?"  (Read in data on dummy write (null MOSI signal)) - only one needed as 1 byte, but it seems like it responses will send a byte back in 16 bit response total, likely this LSB is useless, but for timing it will be collected.  This may always be a duplicate of the first byte,
    
    status = SpiClosePort(0);
    if(status < 0) exit_failure();
    
  #endif

	#ifdef ESP32ARCH  //example SPI routine for the ESP32
	  spy = spiStartBus(VSPI, SPI_CLOCK_DIV16, SPI_MODE0, SPI_MSBFIRST); //Setup ESP32 SPI bus
	  spiAttachSCK(spy, -1);
      spiAttachMOSI(spy, -1);
      spiAttachMISO(spy, -1);
      digitalWrite(_SS, LOW); //Bring SS LOW (Active)
      spiTransferByte(spy, commandHeader1); //Send MSB
      spiTransferByte(spy, commandHeader2);  //Send LSB
      one = spiTransferByte(spy, WRITE);  //dummy write MSB, read out MSB
      two = spiTransferByte(spy, WRITE);  //dummy write LSB, read out LSB
      digitalWrite(_SS, HIGH);  //Bring SS HIGH (inactive)
      spiStopBus(spy);
	#endif

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
     printf(" ADE9078::spiRead16 function details: \n");
     printf(" Command Header: \n");
     printf(commandHeader1, BIN);
     printf(commandHeader2, BIN);
     printf(" Address Byte 1(MSB)[HEX]: \n");
     printf(" Returned bytes (1(MSB) and 2) [HEX]: \n");
     printf("%02x", HEX); //print MSB
     printf(" \n");
     printf(two, HEX);  //print LSB
     printf(" ADE9078::spiRead16 function completed \n");
    #endif

	readval_unsigned = (one << 8);  //Process MSB  (Alternate bitshift algorithm)
    readval_unsigned = readval_unsigned + two;  //Process LSB
	return readval_unsigned;
}



int main()
{
    int fd = SpiOpenPort(0);

    expander_t *exp = expander_init(0x26);
    ADE9078_getVersion();

    SpiClosePort(0);


    return 0;
}