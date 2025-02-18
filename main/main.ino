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

// Task to be run on Core 1
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



// Parse the user input and move the motors
void parseInput(String inputString){

  int index = inputString.indexOf(' ');  // Find the position of the comma

  String inputKey = inputString.substring(0, index);      // "Hello"
  String OtherParts = inputString.substring(index + 1);    // "World"

  // Startup Mode: STARTUP
  if(inputKey == "STARTUP"){
    STATE = "STARTUP";
    Serial.println("In Startup Mode");

      // Set both motors to stopped
      motor1.write(90);                 
      motor2.write(90);                  
  }

  // Manual Mode: MOV
  else if(inputKey == "MOV"){

    String direction = inputString.substring(index + 1); 
    STATE = "MOV";

    Serial.println("// MOV //");

    Serial.print("The state is: ");
    Serial.println(STATE);

    Serial.print("The inputKey is: ");
    Serial.println(inputKey);

    Serial.print("The direction is: ");
    Serial.println(direction);


    // Testing up and down
    if(direction == "w"){
      Serial.println("Forward");
      motor1.write(110);                 
      motor2.write(110);  
    }
    else if(direction == "s"){
      Serial.println("Backward");
      motor1.write(70);                 
      motor2.write(70);  
    }
    else {
      Serial.println("Not sure what the direction is??");
    }
    
    Serial.println("// MOV //");
  }

  // Test Mode: TEST
  else if(inputKey == "TEST"){
    Serial.println("In Test Mode");
  }

  // Base Case
  else {
    Serial.println("Invalid Input");
  }
}

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




