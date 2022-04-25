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

uint16_t ADE9078_spiRead16(uint16_t address, expander_t *exp) { //This is the algorithm that reads from a register in the ADE9078. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
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

uint32_t ADE9078_spiRead32(uint16_t address,expander_t *exp) { //This is the algorithm that reads from a 32 bit register in the ADE9078. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.  Caution, some register elements contain information that is only 24 bit with padding on the MSB
  #ifdef ADE9078_VERBOSE_DEBUG
   printf(" ADE9078::spiRead32 function started \n");
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
    expander_printGPIO(exp);
    expander_resetOnlyPinSetOthersGPIO(exp, 5);
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
   printf(" ADE9078::spiRead32 function details: \n");
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
   printf("%02x", four);
  #endif

  //Post-read packing and bitshifting operations
  return (((uint32_t) one << 24) + ((uint32_t) two << 16) + ((uint32_t) three << 8) + (uint32_t) four);
}


  uint16_t ADE9078_getVersion(expander_t *exp){

	  return ADE9078_spiRead16(VERSION_16, exp);
}


uint32_t ADE9078_getInstVoltageA(expander_t *exp){
	uint32_t value=0;
	value=spiRead32(AV_PCF_32, exp);
return value;
}
int main(){

    expander_t *exp = expander_init(0x27);
    //spi_init();
    // uint8_t send_data = 0x23;
    // uint8_t read_data = bcm2835_spi_transfer(send_data);
    // printf("Sent to SPI: 0x%02X. Read back from SPI: 0x%02X.\n", send_data, read_data);
    // if (send_data != read_data)
    //   printf("Do you have the loopback from MOSI to MISO connected?\n");
    // bcm2835_spi_end();

    

    printf("version %04x\n",ADE9078_getVersion(exp));
    printf("version %04x\n",ADE9078_getInstVoltageA(exp));


    bcm2835_close();

  return 0;
}