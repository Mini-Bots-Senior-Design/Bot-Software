// File to control all onboard software
#include <ESP32Servo.h> 



///////////
// Modes //
///////////
// STARTUP
// MOV
// TEST
// MOVGPS
// MOVPWM
String STATE = "STARTUP";

Servo motor1;  // create servo object to control a servo
Servo motor2; 

int minUs = 1000;
int maxUs = 2000;
int frequency = 50; // 50Hz

int motorpin1 = 13;      // GPIO pin used to connect the servo control (digital out) 
int motorpin2 = 2;      // GPIO pin used to connect the servo control (digital out)  

const char* BOTID = "1";

void setup() {
  // Start communication with the Serial Monitor (USB Serial)
  Serial.begin(115200); // Standard baud rate for Serial Monitor

  motor1.setPeriodHertz(frequency);// Standard 50hz servo
  motor2.setPeriodHertz(frequency);// Standard 50hz servo

  motor1.attach(motorpin1, minUs, maxUs);   // attaches the servo on pin 18 to the servo object
  motor2.attach(motorpin2, minUs, maxUs);   // attaches the servo on pin 18 to the servo object

  xTaskCreatePinnedToCore(
    task1,   // Function to run
    "tast1",   // Name of the task
    1000,            // Stack size (bytes)
    NULL,            // Task input parameter
    1,               // Task priority (1 is low)
    NULL,            // Task handle
    0                // Core to run on (Core 1)
  );

  xTaskCreatePinnedToCore(
    task2,   // Function to run
    "tast2",   // Name of the task
    1000,            // Stack size (bytes)
    NULL,            // Task input parameter
    1,               // Task priority (1 is low)
    NULL,            // Task handle
    0                // Core to run on (Core 1)
  );

  delay(1000); // short delay

  Serial.println("Startup Completed");
}

// Reads the Serial Port
void loop() {
    String inputString = "";  // String to hold the incoming data

    // Read bytes from the serial until a newline character is found
    while (Serial.available() > 0) {

      // TODO: Change this to read from LoRA, or have the option to...
      char incomingByte = Serial.read();  // Read a byte from Serial

      if (incomingByte == '\n') {

        parseInput(inputString);
      
        inputString = "";// Clear the string for the next line
      } else {
        // Otherwise, add the byte to the string
        inputString += incomingByte;
      }
      delay(100);
    }
}


///////////////////////////////////
// {BotID, Command, Parameters} //
//////////////////////////////////

// BotID: {1,2,3,4}
// Command: {MOV, MOVPWM, MOVGPS, STARTUP}

// Parameters: 
//  MOV: {F,L,R,S,B}
//  MOVGPS: {LAT, LAN}
//  MOVPWM: {PWMLeft, PWMRight}
void parseInput(String inputString){

  // Parse Commas
  int indexOfFirstComma = inputString.indexOf(',');   
  int indexOfSecondComma = inputString.indexOf(',', indexOfFirstComma + 1);  

  // Extract "BotID"
  String BotID = inputString.substring(0, indexOfFirstComma);  
  Serial.println("BotID: " + BotID);

  if(BotID == BOTID){

    // Extract Command
    String Command = inputString.substring(indexOfFirstComma + 1, indexOfSecondComma); 
    Serial.println("Command: " + Command);

    // MOV
    if(Command == "MOV"){
      Serial.println("moovin and grovin");

      // Extract Command
      int indexOfThirdComma = inputString.indexOf(',',indexOfSecondComma + 1); 
      String Direction = inputString.substring(indexOfSecondComma + 1, indexOfThirdComma); 
      Serial.println("Direction: " + Direction);

      // Move the Motors
      moveMotorsForMOV(Direction);
    }

    // MOVGPS
    else if(Command == "MOVGPS"){
      Serial.println("moovin with da gpss");

      // TD: Parse the GPS Points
    }

    // MOVPWM
    else if(Command == "MOVPWM"){
      Serial.println("moovin with da da pwmm");

      // TD: Parse the PWM Values
      // TD: Send PWM Values to the Motors
    }

    // STARTUP
    else if(Command == "STARTUP"){
      Serial.println("startin this up");
      motor1.write(90);                 
      motor2.write(90);
    }
  } 
  else{
    Serial.println("Incorrect BotID"); // DEBUG
  }
}

void moveMotorsForMOV(String direction){
  if(direction == "F"){
    Serial.println("Movin Forward");
    motor1.write(110);                 
    motor2.write(110); 
  }
  else if(direction == "B"){
    Serial.println("Movin Back");
    motor1.write(70);                 
    motor2.write(70); 
  }
  else if(direction == "L"){
    Serial.println("Movin Left");
    motor1.write(110);                 
    motor2.write(70); 
  }
  else if(direction == "R"){
    Serial.println("Movin Right");
    motor1.write(70);                 
    motor2.write(110); 
  }
  else if(direction == "S"){
    Serial.println("Movin Stop");
    motor1.write(90);                 
    motor2.write(90); 
  }
  else{
    Serial.println("Incorrect Format");
  }
}

// Reads the GPS Data
void task1(void *parameter) {
  while (true) {
    Serial.println("Reading Data 1");
    vTaskDelay(1000);    
  }
}

void task2(void *parameter) {
  while (true) {
    Serial.println("Reading Data 2");
    vTaskDelay(1000);    
  }
}




