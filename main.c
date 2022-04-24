#include "exp1-i2c.h"
#include "exp2-i2c.h"
#include "expander-i2c.h"



void test_exp1(){
    
    exp1_t *e = exp1_init();;

    printf("Debut EXP1:\n");


/**
 * basculement du relais NL1
 * 
 **/
    for(int i = 0; i<5; i++)
    {
        printf("toggle de 0\n");
        exp1_togglePinGPIO(e,0);
        exp1_printGPIO(e);
        sleep(1);

    }
    



    exp1_closeAndFree(e);
    printf("Fin !\n\n");
}


void test_exp2(){
    
    exp2_t *e = exp2_init();

    exp2_openI2C(e);
    exp2_setI2C(e);

    //exp2_getGPIO();

    printf("Debut EXP2:\n");
    exp2_printGPIO(e);

    printf("set de 7\n");
    exp2_setPinGPIO(e, 7);
    exp2_printGPIO(e);
    

    printf("set de 6\n");
    exp2_setPinGPIO(e, 6);
    exp2_printGPIO(e);

    printf("set de 5\n");
    exp2_setPinGPIO(e, 5);
    exp2_printGPIO(e);

    printf("reset de 6\n");
    exp2_resetPinGPIO(e, 6);
    exp2_printGPIO(e);

    printf("toggle de 1\n");
    exp2_togglePinGPIO(e, 1);
    exp2_printGPIO(e);


    exp2_closeAndFree(e);
    printf("Fin !\n\n");
}



int main()
{
    expander_t *e = expander_init(0x26);

    expander_printGPIO(e);
   
    //printf("%s\n",__func__);
   //test_exp1();
    //test_exp2();

    return 0;
}