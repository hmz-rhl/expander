#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#ifndef SPI_TOOLS_H
#define SPI_TOOLS_H

static void pabort(const char *s);
int  spi_init();
static void transfer(int fd, uint8_t *tx, uint8_t *rx);


#endif