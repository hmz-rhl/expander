#include <wiringPi.h>
#include <wiringPiSPI.h>

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


	int fd = wiringPiSPISetup (0, 1000000);
	if (fd < 0)
	{
		fprintf (stderr, "Can't open the SPI bus: %s\n", strerror (errno)) ;
		exit (EXIT_FAILURE) ;
  	}
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
    
	
	expander_resetOnlyPinSetOthersGPIO(exp, 5);
	wiringPiSPIDataRW (0, data, 6);
	expander_setAllPinsGPIO(exp);


	printf("id bytes : ");
	for (size_t i = 0; i < 6; i++)
	{
		printf("%02x ", data[i]);

	}
	putchar('\n');
	expander_printGPIO(exp1);
	close(fd);

	return EXIT_SUCCESS;
}