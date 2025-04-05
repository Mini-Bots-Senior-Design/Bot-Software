// File to control all onboard software
#include <ESP32Servo.h> 
#include <math.h> // GPS Functions

#include "LoRaBoards.h" // LoRa 
#include <RadioLib.h>   // LoRa


// #include <Wire.h> //Needed for I2C to GNSS
// #include <SparkFun_u-blox_GNSS_v3.h> // GPS
#include "SparkFun_BNO08x_Arduino_Library.h" // IMU


#define EARTH_RADIUS_FEET 20902230.0 // Earth's radius in feet      // GPS Functions
#define RAD_TO_DEG (180.0 / M_PI) //    Convert radians to degrees  // GPS Functions

// Pin configuration for T-Beam SX1278
#define RADIO_CS_PIN 18
#define RADIO_DIO0_PIN 26
#define RADIO_RST_PIN 23
#define RADIO_DIO1_PIN 33
#define RADIO_DIO2_PIN 32  // Optional


Servo motor1;  // create servo object to control a servo
Servo motor2; 

int minUs = 1000;
int maxUs = 2000;
int frequency = 50; // 50Hz

int motorpin1 = 13;      // GPIO pin used to connect the servo control (digital out) 
int motorpin2 = 2;      // GPIO pin used to connect the servo control (digital out)  

const char* BOTID = "1";

// Sensor Values
long Current_GPS_Latitude = 0;
long Current_GPS_Longitude = 0;
float Current_Compass_Heading = 0;

long Target_GPS_Latitude = 0;
long Target_GPS_Longitude = 0;

// State
bool automaticMode = false;

// LoRa Start
// Set up the LoRa module for SX1278
SX1278 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);

String payload = "";
volatile bool receivedFlag = false;
volatile bool transmittedFlag = false;

// Mutexes to protect shared resources
SemaphoreHandle_t xMutexSensor_IMU;
SemaphoreHandle_t xMutexSensor_GPS;

SemaphoreHandle_t xMutexAutomatic;

SemaphoreHandle_t xSemaphore; // LoRa



void setup() {
  // Start communication with the Serial Monitor (USB Serial)
  Serial.begin(115200); 

  // Motor Setup
  motor1.setPeriodHertz(frequency);// Standard 50hz servo
  motor2.setPeriodHertz(frequency);// Standard 50hz servo
  motor1.attach(motorpin1, minUs, maxUs);   // attaches the servo on pin 18 to the servo object
  motor2.attach(motorpin2, minUs, maxUs);   // attaches the servo on pin 18 to the servo object

  // Create Tasks, 

  delay(1000); // short delay

  // Calibrate Motors
  Serial.println("Calibrating Motors...");
  motor1.write(90);                 
  motor2.write(90);
  delay(5000); 
  Serial.println("Calibration Finished");

  setupLoRa();

  xSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphore);

  // For sensors
  xMutexSensor_IMU = xSemaphoreCreateMutex();
  xMutexSensor_GPS = xSemaphoreCreateMutex();

  xMutexAutomatic = xSemaphoreCreateMutex();


  xTaskCreate(BotTask, "Bot Task", 2048, NULL, 1, NULL);

  xTaskCreate(IMUTask, "IMU Task", 2048, NULL, 1, NULL);
  xTaskCreate(GPSTask, "GPS Task", 2048, NULL, 1, NULL);

  xTaskCreate(AutomaticTask, "Automatic Task", 2048, NULL, 1, NULL);

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
 

/////////////////
// Tasks Start // 
/////////////////

// Bot Task: Listen for requests, then send ID when requested
void BotTask(void *pvParameters) {

  switchToReceiveMode();  // Start in receive mode
  String receivedMessage;

  while (1) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      int state = radio.receive(receivedMessage);

      if (state == RADIOLIB_ERR_NONE && receivedFlag) {
        Serial.println("Received: " + receivedMessage);
        unsigned long sendStart = millis();

        if(parseInput(receivedMessage)){
          // Reset and setup trasmit setting
          transmittedFlag = false;
          switchToTransmitMode();

          // Setup Payload
          payload = createPayload();

          int transmissionState = radio.transmit(payload);
        }

        // Reset and setup receive setting
        switchToReceiveMode();  
        receivedFlag = false;
      }
      xSemaphoreGive(xSemaphore);
      vTaskDelay(100 / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
    }
  }
}


// This will sample the GPS and IMU Sensors and write them to global variables 
void GPSTask(void *pvParameters){

  // These store local readings
  long local_GPS_Latitude = 9; // change to store values
  long local_GPS_Longitude = 10; // change to store values

  while(1){
    if (xSemaphoreTake(xMutexSensor_GPS, portMAX_DELAY) == pdTRUE) {

      // write local values to globals
      Current_GPS_Latitude = local_GPS_Latitude;
      Current_GPS_Longitude = local_GPS_Longitude;

      // Give the mutex back so other tasks can use it
      xSemaphoreGive(xMutexSensor_GPS);
    
    }
    else {
      Serial.println("Failed to take mutex");
    }

    vTaskDelay(25 / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
  }
}

void IMUTask(void *pvParameters){

  // These store local readings
  float local_Compass_Heading = 1.1; // change to store values

  while(1){
    if (xSemaphoreTake(xMutexSensor_IMU, portMAX_DELAY) == pdTRUE) {

      // write local values to globals
      Current_Compass_Heading = local_Compass_Heading;

      // Give the mutex back so other tasks can use it
      xSemaphoreGive(xMutexSensor_IMU);
    }
    else {
      Serial.println("Failed to take mutex");
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
  }
}

void AutomaticTask(void *pvParameters){
  while(1){
    if(getAutomaticMode()){
      Serial.println("In Automatic Mode");

      String gpsString = ""; // Debug

      long targetLat = Target_GPS_Latitude; // assuming no mutex conflict
      long targetLon = Target_GPS_Longitude; // assuming no mutex conflict

      long currentLat;
      long currentLon;

      float currentCompass;

      if (xSemaphoreTake(xMutexSensor_GPS, portMAX_DELAY) == pdTRUE) {
        currentLat = Current_GPS_Latitude;
        currentLon = Current_GPS_Longitude;
        xSemaphoreGive(xMutexSensor_GPS);
      }

      if (xSemaphoreTake(xMutexSensor_IMU, portMAX_DELAY) == pdTRUE) {
        currentCompass = Current_Compass_Heading;
        xSemaphoreGive(xMutexSensor_IMU);
      }

      // DEBUG Print:
      // Convert the longitude and latitude to a string and concatenate them
      // gpsString = String(currentLon) + "," + String(currentLat);
      // Serial.print("Current GPS/Compass: ");
      // Serial.print(gpsString);
      // Serial.print(" ,Compass: ");
      // Serial.println(String(currentCompass));

      // gpsString = "Longitude: " + String(targetLon) + ", Latitude: " + String(targetLat);
      // Serial.print("Target GPS: ");
      // Serial.println(gpsString);

      // TD: make the calculations
      // TD: move the motors
    }
    else{
      Serial.println("Not In Automatic Mode");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
  }
}


///////////////
// Tasks End // 
///////////////
void setupLoRa() {
    setupBoards();

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("LoRa init failed, code: "));
        Serial.println(state);
        while (true);
    }

    radio.setFrequency(433.0);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(8);
    radio.setCodingRate(6);
    radio.setSyncWord(0x12);
    radio.setOutputPower(17);

    //radio.implicitHeader(255); //ONLY FOR SF6 (param is max packet size)

    radio.setPacketSentAction(setTransmissionFlag);
    radio.setPacketReceivedAction(setReceiverFlag);
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
bool parseInput(String inputString){

  // Parse Commas
  int indexOfFirstComma = inputString.indexOf(',');   
  int indexOfSecondComma = inputString.indexOf(',', indexOfFirstComma + 1);  

  // Extract "BotID"
  String BotID = inputString.substring(0, indexOfFirstComma);  
  // Serial.println("BotID: " + BotID);

  if(BotID == BOTID){

    // Extract Command
    String Command = inputString.substring(indexOfFirstComma + 1, indexOfSecondComma); 


    if(Command == "MOVAUTO"){

      // Parse GPS Points
      
      // parse lat
      int indexOfThirdComma = inputString.indexOf(',',indexOfSecondComma + 1); 
      String latString = inputString.substring(indexOfSecondComma + 1, indexOfThirdComma); 

      // parse lon
      int indexOfFourthComma = inputString.indexOf(',',indexOfThirdComma + 1); 
      String lonString = inputString.substring(indexOfThirdComma + 1, indexOfFourthComma); 

      // Set Target Points
      setTargetGPSPoints(latString.toInt(), lonString.toInt());

      setAutomaticModeTrue();
    } else{
      setAutomaticModeFalse();
    }

    // MOVPWM
    if(Command == "MOVPWM"){
      // Serial.println("MOVPWM");

      // parse leftPWM
      int indexOfThirdComma = inputString.indexOf(',',indexOfSecondComma + 1); 
      String leftPWMString = inputString.substring(indexOfSecondComma + 1, indexOfThirdComma); 

      // parse rightPWM
      int indexOfFourthComma = inputString.indexOf(',',indexOfThirdComma + 1); 
      String rightPWMString = inputString.substring(indexOfThirdComma + 1, indexOfFourthComma); 

      moveMotorsForMOVPWM(leftPWMString.toInt(), rightPWMString.toInt());
    }

    // STARTUP
    if(Command == "STARTUP"){
      // Serial.println("STARTUP");
      stopMotors();
    }
    
    return true;
  } 
  else{
    Serial.println("Incorrect BotID"); // DEBUG
    return false;
  }
}


///////////////////// 
// Motor Functions // 
/////////////////////

void stopMotors(){
  motor1.write(90);                 
  motor2.write(90);
}

void moveMotorsForMOVPWM(int leftPWM, int rightPWM){
  Serial.println("Movin Via da PWM");
  motor1.write(leftPWM);                 
  motor2.write(rightPWM); 
}



////////////////////
// LoRa Functions //
////////////////////

// Callback functions for RadioLib events
void setReceiverFlag() {receivedFlag = true;}
void setTransmissionFlag() {transmittedFlag = true;}

void switchToTransmitMode() {
    radio.standby();  // Ensure module is in standby before transmit
    transmittedFlag = false;
    //Serial.println("Switching to Transmit Mode");
}

void switchToReceiveMode() {
    radio.standby();
    receivedFlag = false;
    radio.startReceive();  // Start listening
    //Serial.println("Switching to Receive Mode");
}


//////////////////////////
// LoRa Setup Functions //
//////////////////////////

void handleTransmission(bool transmittedFlag, int transmissionState){
  if (transmittedFlag) {
    Serial.println("Bot transmitted: " + payload);
  } else {
    Serial.println("Transmit error, code " + String(transmissionState));
  }
}


//////////////////////////////
// Payload Helper Functions //
//////////////////////////////

// Return a string
String getGPS_String() {
  long currentLon;
  long currentLat;

  if (xSemaphoreTake(xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
    currentLon = Current_GPS_Longitude;
    currentLat = Current_GPS_Latitude;
    xSemaphoreGive(xMutexAutomatic);
  }

  // Convert the longitude and latitude to a string and concatenate them
  String gpsString = String(currentLat) + "," + String(currentLon);
  Serial.println(gpsString);
  return gpsString;  // Return the formatted string
}

String createPayload(){

  String latLon = getGPS_String();
  String createdString = "0," + String(BOTID) + "," + latLon; // Why Zero? ohh zero is SBC

  Serial.print("Payload created: ");
  Serial.println(createdString);

  return createdString;
}


////////////////////////////////////
// Automatic Mode Setters/Getters //
////////////////////////////////////

bool getAutomaticMode(){
  bool localAutomaticMode;

  if (xSemaphoreTake(xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
    localAutomaticMode = automaticMode;

    xSemaphoreGive(xMutexAutomatic);
  }
   else {
    Serial.println("Failed to take mutex");
  }

  return localAutomaticMode;
}

void setAutomaticModeFalse(){
  if (xSemaphoreTake(xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
    automaticMode = false;

    xSemaphoreGive(xMutexAutomatic);
  }
   else {
    Serial.println("Failed to take mutex");
  }
}

void setAutomaticModeTrue(){
  if (xSemaphoreTake(xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
    automaticMode = true;

    xSemaphoreGive(xMutexAutomatic);
  }
   else {
    Serial.println("Failed to take mutex");
  }
}


//////////////////////////
// GPS Helper Functions //
//////////////////////////

void setTargetGPSPoints(long lat, long lon){
    // Assume no Mutex Conflict

    Target_GPS_Longitude = lon;  // Longitude
    Target_GPS_Latitude = lat;   // Latitude 
}

// Function to convert degrees * 10^-7 to decimal degrees
double convertToDecimal(long value) {
    return value / 10000000.0;
}

// Haversine formula to compute distance in feet
double haversineDistance(long lat1, long lon1, long lat2, long lon2) {
  // Convert coordinates to decimal degrees
  double lat1_d = convertToDecimal(lat1);
  double lon1_d = convertToDecimal(lon1);
  double lat2_d = convertToDecimal(lat2);
  double lon2_d = convertToDecimal(lon2);

  // Convert decimal degrees to radians
  double lat1_rad = lat1_d * M_PI / 180.0;
  double lon1_rad = lon1_d * M_PI / 180.0;
  double lat2_rad = lat2_d * M_PI / 180.0;
  double lon2_rad = lon2_d * M_PI / 180.0;

  // Haversine formula
  double dlat = lat2_rad - lat1_rad;
  double dlon = lon2_rad - lon1_rad;
  
  double a = pow(sin(dlat / 2), 2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS_FEET * c; // Distance in feet
}


// Calculate the initial bearing (angle) between two GPS points
double calculateBearing(long lat1, long lon1, long lat2, long lon2) {
  // Convert to decimal degrees
  double lat1_d = convertToDecimal(lat1);
  double lon1_d = convertToDecimal(lon1);
  double lat2_d = convertToDecimal(lat2);
  double lon2_d = convertToDecimal(lon2);

  // Convert to radians
  double lat1_rad = lat1_d * M_PI / 180.0;
  double lon1_rad = lon1_d * M_PI / 180.0;
  double lat2_rad = lat2_d * M_PI / 180.0;
  double lon2_rad = lon2_d * M_PI / 180.0;

  // Compute differences
  double deltaLon = lon2_rad - lon1_rad;

  // Bearing formula
  double y = sin(deltaLon) * cos(lat2_rad);
  double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(deltaLon);
  double theta = atan2(y, x); // Angle in radians

  // Convert to degrees
  double bearing = theta * RAD_TO_DEG;

  // Normalize to 0-360 degrees
  if (bearing < 0) {
      bearing += 360.0;
  }

  return bearing;
}










