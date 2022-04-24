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
#include <stdarg.h>

#define I2C_DEVICE          "/dev/i2c-1"
//#define EXP2_ADDR       (0x27)
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

typedef struct expander
{
    /* data */
    int fd;
    uint8_t buff[4];
    char label[8][MAX_STRING];
    uint8_t addr;

}expander_t;

expander_t* expander_init(uint8_t);

void expander_labelize(expander_t*);

void expander_openI2C(expander_t*);

void expander_closeI2C(expander_t*);

void expander_setI2C(expander_t*);

uint8_t expander_getAllGPIO(expander_t*);
uint8_t expander_getPinGPIO(expander_t*, uint8_t);

void expander_setPinGPIO(expander_t*, uint8_t);
void expander_resetPinGPIO(expander_t*, uint8_t);

void expander_setOnlyPinResetOthersGPIO(expander_t*, uint8_t);
void expander_resetOnlyPinSetOthersGPIO(expander_t*, uint8_t);

void expander_togglePinGPIO(expander_t*, uint8_t);

void expander_resetAllGPIO(expander_t*);
void expander_setAllGPIO(expander_t*);


void expander_printGPIO(expander_t*);

void expander_closeAndFree(expander_t*);

