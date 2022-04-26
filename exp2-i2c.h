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
#define EXP2_ADDR       (0x27)
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

typedef struct exp2
{
    /* data */
    int fd;
    uint8_t buff[4];
    char branchement[8][MAX_STRING];

}exp2_t;

exp2_t* exp2_init(void);

void exp2_branchement(exp2_t*);

void exp2_openI2C(exp2_t*);

void exp2_closeI2C(exp2_t*);

void exp2_setI2C(exp2_t*);

uint8_t exp2_getAllGPIO(exp2_t*);
uint8_t exp2_getPinGPIO(exp2_t*, uint8_t);

void exp2_setPinGPIO(exp2_t*, uint8_t);
void exp2_resetPinGPIO(exp2_t*, uint8_t);

void exp2_setOnlyPinResetOthersGPIO(exp2_t*, uint8_t);
void exp2_resetOnlyPinSetOthersGPIO(exp2_t*, uint8_t);

void exp2_togglePinGPIO(exp2_t*, uint8_t);

void exp2_resetAllGPIO(exp2_t*);
void exp2_setAllGPIO(exp2_t*);


void exp2_printGPIO(exp2_t*);

void exp2_closeAndFree(exp2_t*);

