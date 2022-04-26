#include "../lib/expander-i2c.h"


int main(){
    
    expander_t *e = expander_init(0x26);
    

    expander_printGPIO(e);

    
    expander_togglePinGPIO(e,0);
    expander_printGPIO(e);

    expander_closeAndFree(e);
   
    //printf("%s\n",__func__);
    //test_exp1();
    //test_exp2();

    return 0;
}
