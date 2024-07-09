// Receive with start- and end-markers combined with parsing

#include <math.h>
#include <ArduinoQueue.h>

#define QUEUE_SIZE 10

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use by strtok() function

      // variables to hold the parsed data
char messageFromPC[numChars] = "0";
int integerFromPC = 0;
float floatFromPC = 0.0;

String commandPacket;

boolean newData = false;

ArduinoQueue<String> queue(QUEUE_SIZE);

//============

void setup() {
    Serial.begin(9600);
    Serial.println("Start Loop");

    // while (!Serial.available() && millis() < 5000) ;  // wait up to 5 seconds for a serial monitor
    // if (CrashReport) {
    //     Serial.print(CrashReport);
    //     // sometimes if I am hitting repeating crashes... might add
    //     // while (Serial.read() == -1);  // wait for someone to enter something
    //     // while (Serial.read() != -1) ;  // and eat up the line.
    // }
}

//============

void loop() {

    
  // Serial.println("debug");
  recvWithStartEndMarkers();
  commandPacket = commandParser();
  // queue.enqueue(commandPacket);

  // if(queue.isEmpty() != 1){
  //     String temp = queue.getHead();
  //     // Serial.println(queue.getHead());
  //     if(temp[0] == "0"){//opposite
  //       Serial.print("Command: "); Serial.println(queue.dequeue());
  //     }
  //   }
    // Serial.println(queue.isEmpty());

    

    // while(1);

    delay(500);
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    // char startMarker = '<';
    char endMarker = '\n';
    char rc;

    // if(Serial.available() > 0){
    //   recvInProgress = true;
    // }
    // Serial.println(Serial.available());
    // Serial.println("here");
    
    while (Serial.available() > 0 && newData == false) {
        // while(1);
        rc = Serial.read();

        // if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        // }

        // else if () {
        //     recvInProgress = true;
        // }
    }
}

//============

void parseData() {

      // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars," ");      // get the first part - the string
    // Serial.print(strtokIndx); Serial.print("-"); Serial.println(strtokIndx != NULL);
    if(NULL != strtokIndx)
    {
      strcpy(messageFromPC, strtokIndx);

    }

    strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
    
    if(NULL != strtokIndx)
    {
      floatFromPC = atof(strtokIndx);     // convert this part to an integer
    }

    strtokIndx = strtok(NULL, " ");
    if(NULL != strtokIndx)
    {
    integerFromPC = atoi(strtokIndx);   // convert this part to an integer
    }

    


    
}



//============

void showParsedData() {
    Serial.print("Command: ");
    Serial.print(messageFromPC);
    Serial.print(", FloatArg: ");
    Serial.print(floatFromPC);
    Serial.print(", IntArg ");
    Serial.print(integerFromPC);
    Serial.println();
}

String commandParser(){
  if (newData == true) { 
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() replaces the commas with \0
        // Serial.print(Serial.available()); Serial.print(": "); Serial.println(receivedChars);
        // if(Serial.available() > 0){
        //   while(1);
        // }
        // Serial.println(tempChars);

        parseData();
        // showParsedData();
        
        // int code = 0;
        String dat = "0,000.00";

        if(strcmp(messageFromPC,"ping") == 0){
          // code = 1;
          dat = "1,000.00";
          Serial.print(dat); Serial.print(": ");
          Serial.println("pong");
          
        }
        else if(strcmp(messageFromPC,"ledon") == 0){
          // code = 2;
          floatFromPC = fmodf(floatFromPC, 10.0f);
          dat = "2," + (String) floatFromPC;
          dat.c_str();
          Serial.print(dat); Serial.print(": ");
          Serial.print("LED on, intensity: "); Serial.println(floatFromPC);
          
        }
        else if(strcmp(messageFromPC,"ledoff") == 0){
          // code = 3;
          dat = "3,000.00";
          Serial.print(dat); Serial.print(": ");
          Serial.println("LED off");
          
        }
        else if(strcmp(messageFromPC,"dangle") == 0){
          // code = 4;
          floatFromPC = abs(fmodf(floatFromPC, 360.0f));
          dat = "4," + (String) floatFromPC;
          Serial.print(dat); Serial.print(": ");
          Serial.print("Set driver angle to: "); Serial.println(floatFromPC);
          
        }
        else if(strcmp(messageFromPC,"sdwrite") == 0){
          // code = 5;
          dat = "5,000.00";
          Serial.print(dat); Serial.print(": ");
          Serial.println("Start DAQ write to SD");
          
        }
        else if(strcmp(messageFromPC,"sdstop") == 0){
          // code = 6;
          dat = "6,000.00";
          Serial.print(dat); Serial.print(": ");
          Serial.println("Stop DAQ write to SD");
          
        }
        else {
          String dat = "0,000.00";
          Serial.print(dat); Serial.print(": ");
          Serial.print("Error: "); Serial.print(messageFromPC); Serial.println(" is not a valid command");
          
        }
        strcpy(receivedChars,"0");
        // Serial.flush();

        newData = false;
        return dat;
        
    }

}