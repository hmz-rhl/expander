#include "expander-i2c.h"
#include "spi-lib.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>


#define VERSION_16 0x4FE //Reset: 0x0040 Access: R

#define ADE9078_VERBOSE_DEBUG

int spi_cs0_fd;				//file descriptor for the SPI device
int spi_cs1_fd;				//file descriptor for the SPI device
unsigned char spi_mode;
unsigned char spi_bitsPerWord;
unsigned int spi_speed;

const uint8_t WRITE = 0b00000000; //This value tells the ADE9078 that data is to be written to the requested register.
const uint8_t READ = 0b10000000;  //This value tells the ADE9078 that data is to be read from the requested register.

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
 
int SpiWriteAndRead (int SpiDevice, uint8_t *TxData, uint8_t *RxData, int Length, int LeaveCsLow)
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


uint8_t functionBitVal(uint16_t addr, uint8_t byteVal)
{
//Returns as integer an address of a specified byte - basically a byte controlled shift register with "byteVal" controlling the byte that is read and returned
  uint16_t x = ((addr >> (8*byteVal)) & 0xff);

  #ifdef ADE9078_VERBOSE_DEBUG
   printf(" ADE9078::functionBitVal function (separates high and low command bytes of provided addresses) details: ");
   printf(" Address input (dec): ");
   printf("%d\n",addr);
   printf(" Byte requested (dec): ");
   printf("%d\n", byteVal);
   printf(" Returned Value (dec): ");
   printf("%d\n", x);
   printf(" Returned Value (HEX): ");
   printf("%02x\n", x);
   printf(" ADE9078::functionBitVal function completed ");
  #endif

  return x;
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

  uint8_t tx_data[4];
  tx_data[0] = commandHeader1;
  tx_data[1] = commandHeader2;
  tx_data[2] = WRITE;
  tx_data[3] = WRITE;

  uint8_t rx_data[128];

    expander_resetOnlyPinSetOthersGPIO(exp, 5);
    sleep(10);
    Transfer_spi_buffers(fd, tx_data, rx_data,16);
    sleep(10);
    expander_setPinGPIO(exp,5);

  #ifdef RASPBERRYPIZ //Arduino SPI Routine

    int status = SpiOpenPort(0);
    if(status < 0) exit_failure();


    Transfer_spi_buffers(0,tx_data,rx_data,128,0);

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
     printf("%02x\n",commandHeader1);
     printf("%02x\n",commandHeader2);
     printf(" Address Byte 1(MSB)[HEX]: \n");
     printf(" Returned bytes (1(MSB) and 2) [HEX]: \n");
     printf("%02x", one); //print MSB
     printf("\n");
     printf("%02x\n", two);  //print LSB
     printf(" ADE9078::spiRead16 function completed \n");
    #endif

	readval_unsigned = (one << 8);  //Process MSB  (Alternate bitshift algorithm)
    readval_unsigned = readval_unsigned + two;  //Process LSB
	return readval_unsigned;
}


uint8_t ADE9078_getVersion(expander_t *exp, int fd){
	return ADE9078_spiRead16(VERSION_16, exp, fd);
}

int main()
{
  int fd = open("/dev/spidev0.0",O_RDWR);

  spi_config_t spi_config = {
    spi_mode = 0;      // [0-3]  (-1 when not configured).
    lsb_first = 0;     // {0,1}  (-1 when not configured).
    bits_per_word = 8; // [7...] (-1 when not configured).
    spi_speed = 1000000;     // 0 when not configured.
    spi_ready = 1;     // {0,1}  (-1 when not configured).
  }
  expander_t *exp = expander_init(0x26);
  ADE9078_getVersion(exp, fd);

  close(fd);


  return 0;
}