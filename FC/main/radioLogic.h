//radio logic code

class radioLogic {

    //Pin selection, change accordingly
      // SX1278 has the following connections:
      // NSS pin:   10
      // DIO0 pin:  2
      // NRST pin:  9
      // DIO1 pin:  3
      SX1276 radio = new Module(12, 2, 9, 3)

      // save transmission states between loops
    int transmissionState = RADIOLIB_ERR_NONE;

    // flag to indicate transmission or reception state
    bool transmitFlag = false;

    // flag to indicate that a packet was sent or received
    volatile bool operationDone = false;

    void setFlag(void) {
      // we sent or received  packet, set the flag
      operationDone = true;
    }

    void initializeRadio(){

        

    }

};