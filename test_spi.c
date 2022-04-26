#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "expander-i2c.h"
#include "bcm2835.h"

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

#define PART_ID_16 0x403

// test sur la RFID
#define CIU_VERSION 0x6327
#define STATUS 0xA9
#define STATUS_READ 0x03
//#define HUBLOAD



int main(){

	wiringPiSetup();
	if (fd < 0)
	{
		fprintf (stderr, "Can't open the SPI bus: %s\n", strerror (errno)) ;
		exit (EXIT_FAILURE) ;
  	}

#ifdef HUBLOAD

	expander_t *exp = expander_init(0x27);
	expander_t *exp1 = expander_init(0x26);

	uint8_t cmd_hdr1 = (PART_ID_16 >> 4)  & 0xFF ; // on obtient l'octet msb
	uint8_t cmd_hdr2 = ((PART_ID_16 << 4) & 0xF0) | 0x08; // on indique quon souhaite lire en mettant le bit 3 a un (mask)
	uint8_t data[6];


	data[0]= cmd_hdr1;
	data[1]= cmd_hdr2;
	data[2]= 0x00;
	data[3]= 0x00;
	data[4]= 0x00;
	data[5]= 0x00;
    
	
	expander_resetPinGPIO(exp, PM_CS);
	expander_resetPinGPIO(exp, PM1);
	expander_resetPinGPIO(exp, PM0);

	sleep(3);
	wiringPiSPIDataRW (0, data, 6);
	expander_setPinGPIO(exp, PM_CS);


	printf("id bytes : ");
	for (size_t i = 0; i < 6; i++)
	{
		printf("%02x ", data[i]);

	}
	putchar('\n');
	expander_printGPIO(exp);
#endif
	pinMode(0, OUTPUT);
	digitalWrite(0, LOW);
	delay(2);
	digitalWrite(0, HIGH);
	delay (250);
	uint8_t data, rx;
	data = STATUS_READ;


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
    
	digitalWrite(0, LOW);
	bcm2835_spi_transfernb(&data, &rx, 1);
	digitalWrite(0, HIGH);



	printf("status %d \n", rx);
    bcm2835_close();


	return EXIT_SUCCESS;
}