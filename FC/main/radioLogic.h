
#include <Arduino.h>

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_RST 5 
#define RFM95_CS 10 
#define RFM95_INT 4 


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ   433.0
#define RF95_POWER  13

class RadioLogic{

    public:

    RH_RF95 rf95;
    RadioLogic(){
      RH_RF95 rf95(RFM95_CS, RFM95_INT);
      // RH_RF95 rf95;
      // rf95 = *rad;

    }
    // RH_RF95 rf95;
    void initializeRadio();
    void radioTx();
    

};