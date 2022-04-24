/*

    __________Fonctionnel !___________
    
*/


#include "expander-i2c.h"

expander_t* expander_init(uint8_t addr)
{
    if(addr > 0x27 || addr < 0x20 )
    {
        printf("ERREUR %s : vous avez saisie 0x%02x\nOr addr doit etre entre 0x20 et 0x27 pour l'expander\n",__func__);
        exit(EXIT_FAILURE);
    }
    expander_t* exp = malloc(sizeof(expander_t));
    if (exp == NULL){
        printf("ERREUR %s : allocation echouee\n", __func__);
        exit(EXIT_FAILURE);
    }
    
    // va_list ap;

    // va_start(ap, addr);

    // uint8_t expander_number = va_arg(ap, int);
    // if(expander_number == 1)
    // {

    // }
    // va_end(ap);

    exp->addr = addr;
    expander_labelize(exp);
    expander_openI2C(exp);
    expander_setI2C(exp);

    return exp;
}

void expander_labelize(expander_t* exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

    strcpy(exp->label[0],"LED_DIS#*--------->");
    strcpy(exp->label[1],"CP_DIS#*---------->");
    strcpy(exp->label[2],"PP_CS*------------>");
    strcpy(exp->label[3],"CP_CS*------------>");
    strcpy(exp->label[4],"T_CS*------------->");
    strcpy(exp->label[5],"PM_CS*------------>");
    strcpy(exp->label[6],"PM1*-------------->");
    strcpy(exp->label[7],"PM0*-------------->");

}


void expander_openI2C(expander_t *exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    exp->fd = open(I2C_DEVICE, O_RDWR);
    if(exp->fd < 0) {
        printf("ERREUR d'ouverture l'interface I2C de la RPZ...\n");
        exit(EXIT_FAILURE);
    }
}


void expander_closeI2C(expander_t *exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    if(close(exp->fd) < 0) {
        printf("ERREUR de fermeture l'interface I2C de la RPZ...\n");
        exit(EXIT_FAILURE);
    }
}


void expander_setI2C(expander_t *exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

    if(ioctl(exp->fd,I2C_SLAVE,exp->addr) < 0) {
        printf("ERREUR de setting du l'address l'interface I2C de la RPZ ...\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }

}


uint8_t expander_getAllGPIO(expander_t *exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

/**
 * Selection du registre GPIO de l'expoander
 **/
    exp->buff[0] = REG_GPIO; 
    if(write(exp->fd,exp->buff,1) != 1){
        
        printf("ERREUR d'écriture du registre GPIO sur 0x26\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
/**
 * Lecture du registre GPIO de l'expander
 **/
    if(read(exp->fd,exp->buff,1) != 1) {
        printf("ERREUR de de lecture sur GPIO\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
    
    return exp->buff[0];

}

uint8_t expander_getPinGPIO(expander_t *exp, uint8_t pin){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

    uint8_t ret = expander_getAllGPIO(exp);

    return (ret >> pin) & 0x01;
}


void expander_setPinGPIO(expander_t *exp, uint8_t pin){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    if(pin > 7 || pin < 0){

        printf("pin doit etre entre 0 et 7\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
    uint8_t ancienGPIO = expander_getAllGPIO(exp);
    uint8_t nouveauGPIO = ancienGPIO | (0x01 << pin);

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0x00;

    exp->buff[1] = nouveauGPIO;


    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("ERREUR d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 1 de GPIO[%d]\n", pin);
}

void expander_resetPinGPIO(expander_t *exp, uint8_t pin){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

    if(pin > 7 || pin < 0){

        printf("pin doit etre entre 0 et 7\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
    uint8_t ancienGPIO = expander_getAllGPIO(exp);
    uint8_t nouveauGPIO = ancienGPIO & ~(0x01 << pin);

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0x00;

    exp->buff[1] = nouveauGPIO;


    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("ERREUR d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 0 de GPIO[%d]\n", pin);

}


void expander_togglePinGPIO(expander_t* exp, uint8_t pin){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

     if(expander_getPinGPIO(exp, pin)){

        expander_resetPinGPIO(exp,pin);
    }
    else{
        expander_setPinGPIO(exp,pin);
    }
}


void expander_setAllGPIO(expander_t *exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0xFF;

    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("ERREUR d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 1 de tous les GPIO\n");
}

void expander_resetAllGPIO(expander_t *exp){


    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0x00;

    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("ERREUR d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 0 de tous les GPIO\n");
}


void expander_setOnlyPinResetOthersGPIO(expander_t* exp, uint8_t pin){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = 0x01 << pin;

    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("ERREUR d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 1 du seul GPIO[%d]\n", pin);
}

void expander_resetOnlyPinSetOthersGPIO(expander_t* exp, uint8_t pin){
    
    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = ~(0x01 << pin);

    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("ERREUR d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 1 du seul GPIO[%d]\n", pin);
}



void expander_printGPIO(expander_t *exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

/**
 * Selection du registre GPIO de l'expoander
 **/
    exp->buff[0] = REG_GPIO; 
    if(write(exp->fd,exp->buff,1) != 1){
        
        printf("ERREUR d'écriture du registre GPIO sur 0x26\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
/**
 * Lecture du registre GPIO de l'expander
 **/
    if(read(exp->fd,exp->buff,1) != 1) {
        printf("ERREUR de de lecture sur GPIO\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
/**
 * Affichage des ports GPIO de l'expander
 **/
    printf("___Expander 0x%02x_______________\n", exp->addr);

    for (size_t i = 0; i < 8; i++)
    {
        
        printf("%s GPIO[%d] : %d\r\n",exp->label[i], i, (exp->buff[0] >> i ) & 0x01);
    }
    printf("_______________________________\n");
    putchar('\n');
}

void expander_closeAndFree(expander_t *exp)
{
    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: expander_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    expander_closeI2C(exp);
    free(exp);
}
