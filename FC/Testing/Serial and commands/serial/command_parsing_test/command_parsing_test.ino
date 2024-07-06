// Receive with start- and end-markers combined with parsing

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use by strtok() function

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;

int commandCode = 0;

boolean newData = false;

//============

void setup() {
    Serial.begin(9600);
    // Serial.println("This demo expects 3 pieces of data - text, an integer and a floating point value");
    // Serial.println("Enter data in this style <text,12,24.7>  ");
    Serial.println("Start Loop");
}

//============

void loop() {
    recvWithStartEndMarkers();
    commandCode = commandParser();
    if(commandCode > 0){
      // Serial.println(commandCode);
    }
    
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

    while (Serial.available() > 0 && newData == false) {
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
    strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
  
    strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
    floatFromPC = atof(strtokIndx);

    strtokIndx = strtok(NULL, " ");
    integerFromPC = atoi(strtokIndx);
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

int commandParser(){
  if (newData == true) { 
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() replaces the commas with \0
        parseData();
        // showParsedData();
        
        int code = 0;

        if(strcmp(messageFromPC,"ping") == 0){
          code = 1;
          Serial.print(code); Serial.print(": ");
          Serial.println("pong");
          
        }
        else if(strcmp(messageFromPC,"ledon") == 0){
          code = 2;
          Serial.print(code); Serial.print(": ");
          Serial.print("LED on, intensity: "); Serial.println(floatFromPC);
          
        }
        else if(strcmp(messageFromPC,"ledoff") == 0){
          code = 3;
          Serial.print(code); Serial.print(": ");
          Serial.println("LED off");
          
        }
        else if(strcmp(messageFromPC,"dangle") == 0){
          code = 4;
          Serial.print(code); Serial.print(": ");
          Serial.print("Set driver angle to: "); Serial.println(floatFromPC);
          
        }
        else if(strcmp(messageFromPC,"sdwrite") == 0){
          code = 5;
          Serial.print(code); Serial.print(": ");
          Serial.println("Start DAQ write to SD");
          
        }
        else if(strcmp(messageFromPC,"sdstop") == 0){
          code = 6;
          Serial.print(code); Serial.print(": ");
          Serial.println("Stop DAQ write to SD");
          
        }
        else {
          code = 0;
          Serial.print(code); Serial.print(": ");
          Serial.print("Error: "); Serial.print(messageFromPC); Serial.println(" is not a valid command");
          
        }


        newData = false;
        return code;
        
    }

}