#include <ArduinoQueue.h>

#define QUEUE_SIZE 10

ArduinoQueue<String> queue(QUEUE_SIZE);

volatile bool newData = false;

const byte numChars = 32;
char receivedChars[numChars];

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);

  Serial.println("Start");
}
void loop() {

  char rc;
  int i = 0;
  bool nextLoopPrint = false;
  bool reading = false;

  if(!queue.isEmpty()){
    String toPrint = queue.dequeue();
    Serial.print("Command: "); Serial.println(toPrint);

  }


  // while(Serial.available() > 0 && cont){
  //   rc = Serial.read();
  //   receivedChars[i] = rc;
  //   i++;
  // }
  if(Serial.available() == 0){return;}

  while (Serial.available() > 0){
    if(reading == true){
      Serial.println(Serial.available());
      rc = Serial.read();
      receivedChars[i] = rc;

      if(rc == '\n'){
        nextLoopPrint = true;
      }
    
      i++;
    }
    reading = true;
  }

  if(nextLoopPrint == true){
  Serial.print("Command: "); Serial.println(receivedChars);
  receivedChars[i] = '\0';
  reading = false;
  }
}

int commandParser(String str){
  

}

// void parseData() {

//     // split the data into its parts
    
//   char * strtokIndx; // this is used by strtok() as an index
  
//   strtokIndx = strtok(receivedChars,",");      // get the first part - the string
//   strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
  
//   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
//   integerFromPC = atoi(strtokIndx);     // convert this part to an integer
  
//   strtokIndx = strtok(NULL, ","); 
//   floatFromPC = atof(strtokIndx);     // convert this part to a float

// }