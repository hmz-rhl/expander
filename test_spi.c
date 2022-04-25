#include "wiringPi.h"
#include "wiringPiSPI.h"

#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include <errno.h>
#include <stdint.h>
#include <fcntl.h>


#define MISO 9
#define CS   8
#define SCLK 11
#define MOSI 10

#define PART_ID_16 0x472



int main(){

	uint8_t cmd_hdr1 = (PART_ID_16 & 0xFF0) >> 4; // on obtient l'octet msb
	uint8_t cmd_hdr2 = ((PART_ID_16 & 0x00F) << 4) | 0x08;
	uint8_t data[6];
	data[0]= cmd_hdr1;
	data[1]= cmd_hdr2;
	data[2]= 0x00;
	data[3]= 0x00;
	data[4]= 0x00;
	data[5]= 0x00;
    
	int fd = wiringPiSPISetup (0, 1000000);
	if (fd < 0)
	{
		fprintf (stderr, "Can't open the SPI bus: %s\n", strerror (errno)) ;
		exit (EXIT_FAILURE) ;
  	}
	
	wiringPiSPIDataRW (0, data, 6);

	printf("id bits : %02x\n", data);

	close(fd);

	return EXIT_SUCCESS;
}