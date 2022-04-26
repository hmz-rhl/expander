/*

    __________Fonctionnel !___________
    
*/


#include "exp1-i2c.h"

exp1_t* exp1_init(void)
{
    exp1_t* exp = malloc(sizeof(exp1_t));
    if (exp == NULL || exp == 0){
        printf("erreur d'allocation\n");
        exit(EXIT_FAILURE);
    }
    exp1_branchement(exp);
    exp1_openI2C(exp);
    exp1_setI2C(exp);

    return exp;
}

void exp1_branchement(exp1_t* exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    strcpy(exp->branchement[0], "TYPE-2_NL1_ON*---->");
    strcpy(exp->branchement[1],  "TYPE-2_L2L3_ON*--->");
    strcpy(exp->branchement[2], "TYPE-E/F_ON*------>");
    strcpy(exp->branchement[3], "LOCK_D*----------->");
    strcpy(exp->branchement[4], "RCD_DIS#*--------->");
    strcpy(exp->branchement[5], "RCD_TST#*--------->");
    strcpy(exp->branchement[6], "RCD_RESET#*------->");
    strcpy(exp->branchement[7], "_____------------->");


}


void exp1_openI2C(exp1_t *exp){

     if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    exp->fd = open(I2C_DEVICE, O_RDWR);
    if(exp->fd < 0) {
        printf("ERREUR d'ouverture l'interface I2C de la RPZ...\n");
        exit(EXIT_FAILURE);
    }
}


void exp1_closeI2C(exp1_t *exp){

     if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

    if(close(exp->fd) < 0) {
        printf("ERREUR de fermeture l'interface I2C de la RPZ...\n");
        //exit(EXIT_FAILURE);
    }
}


void exp1_setI2C(exp1_t *exp){

     if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

    if(ioctl(exp->fd,I2C_SLAVE,EXP1_ADDR) < 0) {
        printf("ERREUR de setting du l'address l'interface I2C de la RPZ ...\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }

}


uint8_t exp1_getAllGPIO(exp1_t *exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
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

uint8_t exp1_getPinGPIO(exp1_t *exp, uint8_t pin){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

    uint8_t ret = exp1_getAllGPIO(exp);

    return (ret >> pin) & 0x01;
}


void exp1_setPinGPIO(exp1_t *exp, uint8_t pin){
    
    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    if(pin > 7 || pin < 0){

        printf("pin doit etre entre 0 et 7\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
    uint8_t ancienGPIO = exp1_getAllGPIO(exp);
    uint8_t nouveauGPIO = ancienGPIO | (0x01 << pin);

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = nouveauGPIO;


    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("ERREUR d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 1 de GPIO[%d]\n", pin);

}

void exp1_resetPinGPIO(exp1_t *exp, uint8_t pin){

     if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
        exit(EXIT_FAILURE);
    }

    if(pin > 7 || pin < 0){

        printf("pin doit etre entre 0 et 7\n");
        close(exp->fd);
        exit(EXIT_FAILURE);
    }
    uint8_t ancienGPIO = exp1_getAllGPIO(exp);
    uint8_t nouveauGPIO = ancienGPIO & ~(0x01 << pin);

/* Ecriture des gpio de l'expander
 **/
    exp->buff[0] = REG_OLAT;
    exp->buff[1] = nouveauGPIO;


    printf("ecriture sur OLAT de 0x%02x...\n",exp->buff[1]);

    if(write(exp->fd,exp->buff,2) != 2) {
        printf("ERREUR d'ecriture sur OLAT\r\n");
        exit(EXIT_FAILURE);
    }
    printf("mise a 0 de GPIO[%d]\n", pin);

}


void exp1_setAllGPIO(exp1_t *exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
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

void exp1_resetAllGPIO(exp1_t *exp){

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
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


void exp1_togglePinGPIO(exp1_t* exp, uint8_t pin ){

     if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", "exp1");
        exit(EXIT_FAILURE);
    }

    if(exp1_getPinGPIO(exp, pin)){

        exp1_resetPinGPIO(exp,pin);
    }
    else{
        exp1_setPinGPIO(exp,pin);
    }
}


void exp1_setOnlyPinResetOthersGPIO(exp1_t* exp, uint8_t pin){
    
     if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
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

void exp1_resetOnlyPinSetOthersGPIO(exp1_t* exp, uint8_t pin){
    

    if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
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


void exp1_printGPIO(exp1_t *exp){

     if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
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
    printf("___EXP1________________________\n");
    for (size_t i = 0; i < 8; i++)
    {
        
        printf("%s GPIO[%d] : %d\r\n",exp->branchement[i], i, (exp->buff[0] >> i ) & 0x01);
    }
    printf("_______________________________\n");
    putchar('\n');
}


void exp1_closeAndFree(exp1_t *exp)
{

     if(exp == NULL || exp == 0)
    {
        printf("ERREUR fonction %s : parametre exp NULL (utiliser: exp1_t *e = exp1_init())\n", __func__);
        exit(EXIT_FAILURE);
    }
    if(close(exp->fd) < 0) {
        printf("ERREUR de fermeture l'interface I2C de la RPZ...\n");
        free(exp);
        exit(EXIT_FAILURE);
    }
}
