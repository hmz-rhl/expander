/*

    __________Fonctionnel !___________
    
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/i2c-dev.h>

#define I2C_DEVICE          "/dev/i2c-1"
#define MCP23008_ADDR       (0x26)


// registers
#define REG_IODIR   0x00   //!< I/O direction register
#define REG_IPOL    0x01    //!< Input polarity register
#define REG_GPINTEN 0x02 //!< Interrupt-on-change control register
#define REG_DEFVAL  0x03 //!< Default compare register for interrupt-on-change
#define REG_INTCON  0x04 //!< Interrupt control register
#define REG_IOCON   0x05  //!< Configuration register
#define REG_GPPU    0x06   //!< Pull-up resistor configuration register
#define REG_INTF    0x07   //!< Interrupt flag register
#define REG_INTCAP  0x08 //!< Interrupt capture register
#define REG_GPIO    0x09   //!< Port register
#define REG_OLAT    0x0A   //!< Output latch register


int fd;
int ret;
uint8_t buff[4];



int main(int argc, char* argv[]) {


    if(argc != 9){
        printf("Usage: ./set_expander1_gpio <GPIO 7> ... <GPIO 0> \n");
        printf("exemple: ./set_expander1_gpio 0 1 0 1 0 1 1 1\n");
        exit(EXIT_FAILURE);

    }
    if(argc == 2 && (!strcmp(argv[1],"-h") || !strcmp(argv[1],"--help"))){
        printf("Usage: ./set_expander1_gpio -h, --help affiche ce message d'aide \n");
        printf("       ./set_expander1_gpio <GPIO 7> ... <GPIO 0> \n");

        printf("exemple: ./set_expander1_gpio 0 1 0 1 0 1 1 1\n");
        exit(EXIT_FAILURE);

    }

/**
 * Ouverture de l'interface I2C de la RPZ
 **/

    fd = open(I2C_DEVICE, O_RDWR);
    if(fd < 0) {
        printf("probleme d'ouverture l'interface I2C de la RPZ...\n");
        exit(EXIT_FAILURE);
    }
/**
 * Setting de l'address esclave de l'interface I2C de la RPZ
 **/
    if(ioctl(fd,I2C_SLAVE,MCP23008_ADDR) < 0) {
        printf("probleme de setting du l'address l'interface I2C de la RPZ ...\n");
        exit(EXIT_FAILURE);
    }


/**
 * Lecture des gpio de l'expander
 **/
    buff[0] = REG_GPIO;
   if(write(fd,buff,1) != 1) {
        printf("probleme de selection du registre GPIO\n");
        exit(EXIT_FAILURE);
    }

       if(read(fd,buff,1) != 1) {
        printf("probleme de lecture sur GPIO\n");
        exit(EXIT_FAILURE);
    }

/**
 * Affichage des gpio de l'expander sur la console
 **/

    printf("Avant :\n\n");
    for (size_t i = 0; i < 8; i++)
    {
        
        printf("GPIO[%d] : %d\r\n", i, (buff[0] >> i ) & 0x01);
    }
    putchar('\n');

/**
 * Ecriture des gpio de l'expander
 **/
    buff[0] = REG_OLAT;
    buff[1] = 0x00;

/**
 * Recuperation de la configuration souhaite des gpio de l'expander
 **/
    for (size_t i = 0; i < 8; i++)
    {
       buff[1] += (uint8_t)(pow(2,i)*(atoi(argv[8-i])));
    }
    putchar('\n');


    printf("ecriture sur OLAT de 0x%02x...\n",buff[1]);

    if(write(fd,buff,2) != 2) {
        printf("probleme d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
/**
 * Selection du registre GPIIO de l'expander
 **/
    buff[0] = REG_GPIO;
   if(write(fd,buff,1) != 1) {
        printf("probleme de selection du registre GPIO\n");
        exit(EXIT_FAILURE);
    }
/**
 * Lecture des ports GPIO de l'expander
 **/

       if(read(fd,buff,1) != 1) {
        printf("probleme de lecture sur GPIO\n");
        exit(EXIT_FAILURE);
    }
        

/**
 * Affichage des gpio de l'expander sur la console
 **/
    for (size_t i = 0; i < 8; i++)
    {
        
        printf("%d : %d\r\n", i, (buff[0] >> i ) & 0x01);
    }



    close(fd);

    exit(EXIT_SUCCESS);



    

}