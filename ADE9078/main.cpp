//Resampled Wave form buffer for ADE9078 to read and report values (ADE9078_TEST)
//California Plug Load Research Center - 2019
//Enoch Chau 2019


#include "ADE9078.h"
// #include <SPI.h>
// #include <EEPROM.h>
//#include "arduinoFFT.h"


//Architecture Control:
//Make sure you select in the ADE9078.h file the proper board architecture, either Arduino/AVR/ESP8266 or ESP32
//REMINDER: ONLY SELECT THE SINGLE OPTION FOR THE BOARD TYPE YOU ARE USING!
// #define ESP32ARCH

//Define ADE9078 object with hardware parameters specified
#define local_SPI_freq 115200  //Set SPI_Freq at 1MHz  - used for Arduino/AVR configuration
#define local_SS 10  //Set the SS pin for SPI communication as pin 10, typical on Arduino Uno and similar boards

//****WFB settings********
#define WFB_ALL_SEGMENTS 512
#define BURST_MEMORY_BASE 0x801//well, its supposed to be 0x800 but that doesn't work that well for some reason

//arduinoFFT FFT = arduinoFFT();

#define SAMPLES 64             //FFT Total samples input - Must be a power of 2
unsigned int sampling_period_us;  //holder for microseconds for FFT
#define SAMPLING_FREQUENCY 60 //Hz, match rate to match sampling frequency for input data from data source



//Declare object for buffer & FFT values to be received
struct FullResample
{
    uint16_t Ia[WFB_ALL_SEGMENTS];
    uint16_t Va[WFB_ALL_SEGMENTS];
    uint16_t Ib[WFB_ALL_SEGMENTS];
    uint16_t Vb[WFB_ALL_SEGMENTS];
    uint16_t Ic[WFB_ALL_SEGMENTS];
    uint16_t Vc[WFB_ALL_SEGMENTS];
    uint16_t In[WFB_ALL_SEGMENTS];
};

const int readCount = WFB_ALL_SEGMENTS/WFB_RESAMPLE_SEGMENTS; //==8

struct InitializationSettings* is = new InitializationSettings; //define structure for initialized values

ADE9078 myADE9078(local_SS, local_SPI_freq, is); // Call the ADE9078 Object with hardware parameters specified, local variables are copied to private variables inside the class when object is created.

//esp32 arch for spi
//SPIClass * vspi = NULL;

int main() {

    //Need to add:
    //1)Read in EEPROM values, check the "Configured bit", if 1, display, calibration values available, load these calibration values to the ADE9078's RAM
    //2)If 0, then display that no calibration values are present and load default values (Tell user to run calibration firmware)
    //3) Proceed with load and operation of this demo
    //Serial.begin(115200);
    //delay(200);
    is->vAGain=1;
    is->vBGain=1;
    is->vCGain=1;

    is->iAGain=1;
    is->iBGain=1;
    is->iCGain=1;
    is->iNGain=1;

    is->powerAGain=1;
    is->powerBGain=1;
    is->powerCGain=1;

    //Use these settings to configure wiring configuration at stertup
    //FYI: B010=2 in DEC
    //FYI: B100 = 4 in DEC

    //4 Wire Wye configuration - non-Blondel compliant:
    is->vConsel=0;
    is->iConsel=0;

    //Delta, Blondel compliant:
    //is->vConsel=4; //byte value of 100,
    //is->iConsel=0;


    //Please continue for all cases

    //SPI.begin();
    //delay(200);
    myADE9078.initialize(); //Call initialization of the ADE9078 withe default configuration plus options specified
    //EEPROMInit()  //call only once on a virgin chip to "partition" EEPROM for the input type expected moving forward
    //load_data_allfields();  //load EEPROM values
    //delay(200);
    myADE9078.getVersion();

    sampling_period_us = (int)(1000000*(1.0/SAMPLING_FREQUENCY));  //calculate the sampling period in microseconds for the FFT, relative to 1 MHZ
    return 0;



    while (1){


        FullResample fullResample;
        myADE9078.stopFillingBuffer();
        myADE9078.resetFullBufferBit();
        myADE9078.startFillingBuffer();

        bool check = 0;

        while (check != 1){
            usleep(1);
            check = myADE9078.isDoneSampling();
            // Serial.print("wait status: ");
            // Serial.println(check);
        }


        printf("done sampling, start reading");

        myADE9078.spiBurstResampledWFB(BURST_MEMORY_BASE);
        int i = 0;
        //memcpy
        for (int seg=0; seg < WFB_RESAMPLE_SEGMENTS; ++seg){
            
            printf("Loop position: ");
            printf("%d", i);
            printf(", ");
            printf(seg);
            printf("Segment Offset: ");
            int segOffSet = seg + (i*64);
            printf(segOffSet);

            myADE9078.readIrms();
            printf("A, B, C rms (I): ");
            printf(myADE9078.lastReads.irms.a);
            printf(" ");
            printf(myADE9078.lastReads.irms.b);
            printf(" ");
            printf(myADE9078.lastReads.irms.c);
            printf(" ");


            printf("Ia,Va, Ib,Vb, Ic,Vc, In: ");
            printf(myADE9078.lastReads.resampledData.Ia[seg]); printf(" ");
            printf(myADE9078.lastReads.resampledData.Va[seg]); printf(" ");
            printf(myADE9078.lastReads.resampledData.Ib[seg]); printf(" ");
            printf(myADE9078.lastReads.resampledData.Vb[seg]); printf(" ");
            printf(myADE9078.lastReads.resampledData.Ic[seg]); printf(" ");
            printf(myADE9078.lastReads.resampledData.Ic[seg]); printf(" ");
            printf(myADE9078.lastReads.resampledData.In[seg]);
            i++;
        }

        printf("Finished reading from ADE chip.");


    }

}
