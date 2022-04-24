/*

    __________Fonctionnel !___________
    
*/


#include "exp2-i2c.h"


exp2_t* exp2_init(void)
{
    exp2_t* exp = malloc(sizeof(exp2_t));
    if (exp == NULL){
        printf("erreur d'allocation\n");
        exit(EXIT_FAILURE);
    }
    exp2_branchement(exp);
    exp2_openI2C(exp);
    exp2_setI2C(exp);

    return exp;
}

void exp2_branchement(exp2_t* exp){

    strcpy(exp->branchement[0],"LED_DIS#*--------->");
    strcpy(exp->branchement[1],"CP_DIS#*---------->");
    strcpy(exp->branchement[2],"PP_CS*------------>");
    strcpy(exp->branchement[3],"CP_CS*------------>");
    strcpy(exp->branchement[4],"T_CS*------------->");
    strcpy(exp->branchement[5],"PM_CS*------------>");
    strcpy(exp->branchement[6],"PM1*-------------->");
    strcpy(exp->branchement[7],"PM0*-------------->");

}


void exp2_openI2C(exp2_t *exp){
    exp->fd = open(I2C_DEVICE, O_RDWR);
    if(exp->fd < 0) {
        printf("probleme d'ouverture l'interface I2C de la RPZ...\n");
        exit(EXIT_FAILURE);
    }
}


void exp2_closeI2C(exp2_t *exp){
    if(close(exp->fd) < 0) {
        printf("probleme de fermeture l'interface I2C de la RPZ...\n");
        exit(EXIT_FAILURE);
    }
}


void exp2_setI2C(exp2_t *exp){

    if(ioctl(exp->fd,I2C_SLAVE,EXP2_ADDR) < 0) {
        printf("probleme de setting du l'address l'interface I2C de la RPZ ...\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }

}


uint8_t exp2_getAllGPIO(exp2_t *exp){

/**
 * Selection du registre GPIO de l'expoander
 **/
    exp->buff[0] = REG_GPIO; 
    if(write(exp->fd,exp->buff,1) != 1){
        
        printf("probleme d'écriture du registre GPIO sur 0x26\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
/**
 * Lecture du registre GPIO de l'expander
 **/
    if(read(exp->fd,exp->buff,1) != 1) {
        printf("probleme de de lecture sur GPIO\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
    
    return exp->buff[0];

}

uint8_t exp2_getPinGPIO(exp2_t *exp, uint8_t pin){

    uint8_t ret = exp2_getAllGPIO(exp);

    return (ret >> pin) & 0x01;
}


void exp2_setPinGPIO(exp2_t *exp, uint8_t pin){
    if(pin > 7 || pin < 0){

        printf("pin doit etre entre 0 et 7\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
    uint8_t ancienGPIO = exp2_getAllGPIO(exp);
    uint8_t nouveauGPIO = ancienGPIO | (0x01 << pin);

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0x00;

    exp->buff[1] = nouveauGPIO;


    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("probleme d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 1 de GPIO[%d]\n", pin);
}

void exp2_resetPinGPIO(exp2_t *exp, uint8_t pin){

    if(pin > 7 || pin < 0){

        printf("pin doit etre entre 0 et 7\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
    uint8_t ancienGPIO = exp2_getAllGPIO(exp);
    uint8_t nouveauGPIO = ancienGPIO & ~(0x01 << pin);

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0x00;

    exp->buff[1] = nouveauGPIO;


    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("probleme d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 0 de GPIO[%d]\n", pin);

}


void exp2_togglePinGPIO(exp2_t* exp, uint8_t pin){

     if(exp2_getPinGPIO(exp, pin)){

        exp2_resetPinGPIO(exp,pin);
    }
    else{
        exp2_setPinGPIO(exp,pin);
    }
}


void exp2_setAllGPIO(exp2_t *exp){

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0xFF;

    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("probleme d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 1 de tous les GPIO\n");
}

void exp2_resetAllGPIO(exp2_t *exp){

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0x00;

    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("probleme d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 0 de tous les GPIO\n");
}


void exp2_setOnlyPinResetOthersGPIO(exp2_t* exp, uint8_t pin){
    
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0x01 << pin;

    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("probleme d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 1 du seul GPIO[%d]\n", pin);
}

void exp2_resetOnlyPinSetOthersGPIO(exp2_t* exp, uint8_t pin){
    
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = ~(0x01 << pin);

    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("probleme d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 1 du seul GPIO[%d]\n", pin);
}



void exp2_printGPIO(exp2_t *exp){

/**
 * Selection du registre GPIO de l'expoander
 **/
    exp->buff[0] = REG_GPIO; 
    if(write(exp->fd,exp->buff,1) != 1){
        
        printf("probleme d'écriture du registre GPIO sur 0x26\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
/**
 * Lecture du registre GPIO de l'expander
 **/
    if(read(exp->fd,exp->buff,1) != 1) {
        printf("probleme de de lecture sur GPIO\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
/**
 * Affichage des ports GPIO de l'expander
 **/
    printf("___EXP2________________________\n");

    for (size_t i = 0; i < 8; i++)
    {
        
        printf("%s GPIO[%d] : %d\r\n",exp->branchement[i], i, (exp->buff[0] >> i ) & 0x01);
    }
    printf("_______________________________\n");
    putchar('\n');
}

void exp2_close(exp2_t *exp)
{
    close(exp->fd);
}
