/*

    __________Fonctionnel !___________
    
*/

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
#define EXP1_ADDR       (0x26)
#define MAX_STRING          255


// registers
#define MCP23008_IODIR 0x00   //!< I/O direction register
#define MCP23008_IPOL 0x01    //!< Input polarity register
#define MCP23008_GPINTEN 0x02 //!< Interrupt-on-change control register
#define MCP23008_DEFVAL  0x03 //!< Default compare register for interrupt-on-change
#define REG_INTCON 0x04 //!< Interrupt control register
#define REG_IOCON 0x05  //!< Configuration register
#define REG_GPPU 0x06   //!< Pull-up resistor configuration register
#define REG_INTF 0x07   //!< Interrupt flag register
#define REG_INTCAP 0x08 //!< Interrupt capture register
#define REG_GPIO 0x09   //!< Port register
#define REG_OLAT 0x0A   //!< Output latch register

typedef struct exp1
{
    /* data */
    int fd;
    uint8_t buff[4];
    char branchement[8][MAX_STRING];

}exp1_t;

exp1_t* exp1_init(void);

void exp1_branchement(exp1_t*);

void exp1_openI2C(exp1_t*);

void exp1_closeI2C(exp1_t*);

void exp1_setI2C(exp1_t*);

uint8_t exp1_getAllGPIO(exp1_t*);
uint8_t exp1_getPinGPIO(exp1_t*, uint8_t);

void exp1_setPinGPIO(exp1_t*, uint8_t);
void exp1_resetPinGPIO(exp1_t*, uint8_t);

void exp1_setOnlyPinResetOthersGPIO(exp1_t*, uint8_t);
void exp1_resetOnlyPinSetOthersGPIO(exp1_t*, uint8_t);

void exp1_togglePinGPIO(exp1_t*, uint8_t);

void exp1_resetAllGPIO(exp1_t*);
void exp1_setAllGPIO(exp1_t*);


void exp1_printGPIO(exp1_t*);

void exp1_closeAndFree(exp1_t*);
