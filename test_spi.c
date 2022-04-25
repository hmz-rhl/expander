#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "bcm2835.h"

#include "expander-i2c.h"

#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>


#define MISO 9
#define CS   8
#define SCLK 11
#define MOSI 10

#define PART_ID_16 0x472



int main(){


	// int fd = wiringPiSPISetup (0, 1000000);
	// if (fd < 0)
	// {
	// 	fprintf (stderr, "Can't open the SPI bus: %s\n", strerror (errno)) ;
	// 	exit (EXIT_FAILURE) ;
  	// }
	expander_t *exp = expander_init(0x27);

	uint8_t cmd_hdr1 = (PART_ID_16 & 0xFF0) >> 4; // on obtient l'octet msb
	uint8_t cmd_hdr2 = ((PART_ID_16 & 0x00F) << 4) | 0x08; // on indique quon souhaite lire en mettant le bit 3 a un (mask)
	uint8_t data[6];
	uint8_t one, two,three,four; //holders for the read values from the SPI Transfer

	data[0]= cmd_hdr1;
	data[1]= cmd_hdr2;
	data[2]= 0x00;
	data[3]= 0x00;
	data[4]= 0x00;
	data[5]= 0x00;
    
	
	expander_resetOnlyPinSetOthersGPIO(exp, 5);
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
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
    //bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    //bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default

    uint8_t tx[6];
    uint8_t rx[4];

    expander_resetOnlyPinSetOthersGPIO(exp, 5);
    // bcm2835_spi_write(commandHeader2); //Send MSB
    // bcm2835_spi_write(commandHeader1); //Send MSB
    // one = bcm2835_spi_transfer(WRITE);  //dummy write MSB, read out MSB
    // two = bcm2835_spi_transfer(WRITE);  //dummy write LSB, read out LSB
      bcm2835_spi_transfernb(data, rx, 6);
      one = rx[0];  //dummy write MSB, read out MSB
      two = rx[1];  //dummy write LSB, read out LSB
	  three = rx[2]; 
	  four = rx[3];
	expander_setAllPinsGPIO(exp);

	uint32_t readval_unsigned = (one << 24);  //Process MSB  (Alternate bitshift algorithm)
    readval_unsigned = readval_unsigned + (two << 16);  //Process LSB
    readval_unsigned = readval_unsigned + (three << 8);  //Process LSB
    readval_unsigned = readval_unsigned + four;  //Process LSB

	if((readval_unsigned >> 16) & 0x01)
		printf("ADE9004 : yes \n");
	else{
		printf("ADE9004 : no \n");
	}

	if((readval_unsigned >> 20 )& 0x01)
		printf("ADE9000 : yes \n");
	else{
		printf("ADE9000 : no \n");
	}
	
	if((readval_unsigned >> 21) & 0x01)
		printf("ADE73370 : yes \n");
	else{
		printf("ADE73370 : no \n");
	}

	printf("Done ! \n");
	// close(fd);
    bcm2835_close();

	
	return EXIT_SUCCESS;
}