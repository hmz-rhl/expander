#include "spi-lib.h"

/*
 * spidev tools.
 *
 * (c) 2014-2021 Christophe BLAESS <christophe.blaess@logilin.fr>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */




#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))


void pabort(const char *s)
{
        perror(s);
        abort();
}

int  spi_init(){

        
  const char *device = "/dev/spidev0.0";
  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;
  uint32_t speed = 1000000;
  uint16_t delay = 0;
  int fd = open(device,O_RDWR);
  int ret = -1;
  if (fd < 0)
          pabort("can't open device");

  /*
    * spi mode
    */
  ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
  if (ret == -1)
          pabort("can't set spi mode");

  ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
  if (ret == -1)
          pabort("can't get spi mode");

  /*
    * bits per word
    */
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret == -1)
          pabort("can't set bits per word");

  ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
  if (ret == -1)
          pabort("can't get bits per word");

  /*
    * max speed hz
    */
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret == -1)
          pabort("can't set max speed hz");

  ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  if (ret == -1)
          pabort("can't get max speed hz");

  printf("spi mode: %d\n", mode);
  printf("bits per word: %d\n", bits);
  printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
  return fd;
}

void transfer(int fd, uint8_t *tx, uint8_t *rx)
{
        int ret;
        const char *device = "/dev/spidev0.0";
        uint8_t mode = SPI_MODE_0;
        uint8_t bits = 8;
        uint32_t speed = 1000000;
        uint16_t delay = 0;
        struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = ARRAY_SIZE(tx),
                .delay_usecs = delay,
                .speed_hz = speed,
                .bits_per_word = bits,
        };
        printf("tx: %s\nrx: %s\nlen: %d\ndelay: %d\nspeed: %dHz\nbits/word: %d\n\n", tr.tx_buf, tr.tx_buf, tr.len, tr.delay_usecs, tr.speed_hz, tr.bits_per_word);

        ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1)
                pabort("can't send spi message");
}
