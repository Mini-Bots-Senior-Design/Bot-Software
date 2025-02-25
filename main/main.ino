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

// LoRa start
#include "LoRaBoards.h"
#include <RadioLib.h>

// Pin configuration for T-Beam SX1278
#define RADIO_CS_PIN 18
#define RADIO_DIO0_PIN 26
#define RADIO_RST_PIN 23
#define RADIO_DIO1_PIN 33
#define RADIO_DIO2_PIN 32  // Optional

// Set up the LoRa module for SX1278
SX1278 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);

String payload = "";
volatile bool receivedFlag = false;
volatile bool transmittedFlag = false;

SemaphoreHandle_t xSemaphore;
TaskHandle_t BotTaskHandle = NULL;
// LoRa end


void setup() {
  // Start communication with the Serial Monitor (USB Serial)
  Serial.begin(115200); // Standard baud rate for Serial Monitor

  motor1.setPeriodHertz(frequency);// Standard 50hz servo
  motor2.setPeriodHertz(frequency);// Standard 50hz servo

  motor1.attach(motorpin1, minUs, maxUs);   // attaches the servo on pin 18 to the servo object
  motor2.attach(motorpin2, minUs, maxUs);   // attaches the servo on pin 18 to the servo object

  xTaskCreate(
      task1,   // Function to run
      "task1", // Name of the task
      1000,    // Stack size (bytes)
      NULL,    // Task input parameter
      1,       // Task priority (1 is low)
      NULL     // Task handle (optional)
  );

  xTaskCreate(
      task2,   // Function to run
      "task2", // Name of the task
      1000,    // Stack size (bytes)
      NULL,    // Task input parameter
      1,       // Task priority (1 is low)
      NULL     // Task handle (optional)
  );

  delay(1000); // short delay

  Serial.println("Calibrating Motors...");
  motor1.write(90);                 
  motor2.write(90);
  delay(5000); 
  Serial.println("Calibration Finished");

  // LoRa Calibration Start
  setupBoards();  // Assuming this is defined elsewhere

  xSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphore);

  xTaskCreate(BotTask, "Bot Task", 2048, NULL, 1, NULL);
  // LoRa Calibration End

  Serial.println("Startup Completed");
}

// Reads the Serial Port
void loop() {
    // String inputString = "";  // String to hold the incoming data

    // // Read bytes from the serial until a newline character is found
    // while (Serial.available() > 0) {

    //   // TODO: Change this to read from LoRA, or have the option to...
    //   char incomingByte = Serial.read();  // Read a byte from Serial

    //   if (incomingByte == '\n') {

    //     parseInput(inputString);
      
    //     inputString = "";// Clear the string for the next line
    //   } else {
    //     // Otherwise, add the byte to the string
    //     inputString += incomingByte;
    //   }
    //   delay(100);
    // }
}


/////////////////////////////////////////
// INPUT: {BotID, Command, Parameters} //
/////////////////////////////////////////

// BotID: {1,2,3,4}
// Command: {MOV, MOVPWM, MOVGPS, STARTUP}
//  MOV: {F,L,R,S,B}
//  MOVGPS: {LAT, LAN}
//  MOVPWM: {PWMLeft, PWMRight}

// Ex. 1,MOV,L
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

////////////////////
// LoRa Functions //
////////////////////

// Callback functions for RadioLib events
void setReceiverFlag() {
    receivedFlag = true;
}

void setTransmissionFlag() {
    transmittedFlag = true;
}

// Function to initialize LoRa for receiving
void setupReceive() {
    Serial.println("Initializing Receive Mode...");

    radio.reset();
    delay(3000);

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Radio init failed, code: "));
        Serial.println(state);
        while (true);
    }

    radio.setFrequency(433.0);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(12);
    radio.setCodingRate(6);
    radio.setSyncWord(0x12);

    radio.setPacketReceivedAction(setReceiverFlag);
    Serial.println("Receive mode ready.");
    Serial.println("");
    Serial.println("Bot listening for command...");
}

// Function to initialize LoRa for transmitting
void setupTransmit() {
    Serial.println("Initializing Transmit Mode...");

    radio.reset();
    delay(1500);

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Radio init failed, code: "));
        Serial.println(state);
        while (true);
    }

    radio.setFrequency(433.0);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(12);
    radio.setCodingRate(6);
    radio.setSyncWord(0x12);
    radio.setOutputPower(17);  // Max safe power for 433MHz SX1278 // changed from 15 to 17

    radio.setPacketSentAction(setTransmissionFlag);
    Serial.println("Transmit mode ready.");
}

// Bot Task: Listen for requests, then send ID when requested
void BotTask(void *pvParameters) {
    setupReceive();  // Start in receive mode
    String receivedMessage;

    while (1) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {

          int state = radio.receive(receivedMessage);

          if (state == RADIOLIB_ERR_NONE && receivedFlag) {
            Serial.println("Received: " + receivedMessage);

            parseInput(receivedMessage);

            // Reset and setup trasmit setting
            transmittedFlag = false;
            setupTransmit();
            delay(200); // Reduced delay from 1500ms to 200ms

            // Setup Payload, later functionize this
            payload = "0,1,45,45";

            int transmissionState = radio.transmit(payload);
            handleTransmission(transmittedFlag, transmissionState);

            // Reset and setup receive setting
            setupReceive();  
            receivedFlag = false;
          }
          xSemaphoreGive(xSemaphore);
          vTaskDelay(100 / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
        }
    }
}

void handleTransmission(bool transmittedFlag, int transmissionState){
  if (transmittedFlag) {
    Serial.println("Bot transmitted: " + payload);
  } else {
    Serial.println("Transmit error, code " + String(transmissionState));
  }
}




