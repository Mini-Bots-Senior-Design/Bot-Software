// File to control all onboard software
#include <ESP32Servo.h> 
#include <math.h> // GPS Functions

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "LoRaBoards.h" // LoRa 
#include <RadioLib.h>   // LoRa

#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Define GPS Serial Port (ESP32 has multiple hardware serials)
// HardwareSerial SerialGPS(1);  // Use Serial1

// GPS Object
TinyGPSPlus gps;

// Define GPS Module TX/RX Pins
#define GPS_RX_PIN 34  // Connect GPS TX -> ESP32 RX (Receive)
#define GPS_TX_PIN 12  // Connect GPS RX -> ESP32 TX (Transmit)
#define GPS_BAUD_RATE 9600  // Adjust based on your GPS module

#define EARTH_RADIUS_FEET 20902230.0 // Earth's radius in feet      // GPS Functions
#define RAD_TO_DEG (180.0 / M_PI) //    Convert radians to degrees  // GPS Functions

// Pin configuration for T-Beam SX1278
#define RADIO_CS_PIN 18
#define RADIO_DIO0_PIN 26
#define RADIO_RST_PIN 23
#define RADIO_DIO1_PIN 33
#define RADIO_DIO2_PIN 32  // Optional

#define BOT_ID "1"
#define GPS_Period 1000
#define IMU_Period 250
#define Battery_Sensor_Period 300000


class BotData {
public:
  // Constructor: Initialize sensor values and create a mutex.
  BotData():
       _botID("0"),
      _currentGPSLatitude(0),
      _currentGPSLongitude(0),
      _compassHeading(0.0f),
      _targetGPSLatitude(0),
      _targetGPSLongitude(0),
      _batteryLevel(0.0f),
      _automaticMode(false)
  {
    _xMutexSensor_IMU = xSemaphoreCreateMutex();
    _xMutexSensor_Current_GPS = xSemaphoreCreateMutex();
    _xMutexSensor_Target_GPS = xSemaphoreCreateMutex();
    _xMutexAutomatic = xSemaphoreCreateMutex();
    _xMutexSensor_BatterySensor = xSemaphoreCreateMutex();

    if (_xMutexSensor_IMU == NULL) {Serial.println("Failed to create IMU mutex");}
    if (_xMutexSensor_Current_GPS == NULL) {Serial.println("Failed to create current GPS mutex");}
    if (_xMutexSensor_Target_GPS == NULL) {Serial.println("Failed to create target GPS mutex");}
    if (_xMutexAutomatic == NULL) {Serial.println("Failed to create target automatic mutex");}
    if (_xMutexSensor_BatterySensor == NULL) {Serial.println("Failed to create target automatic mutex");}

  }
  
  // 
  // Destructor: Delete the mutex.
  // ~SensorData() {
  //   if (_mutex != NULL) {
  //     vSemaphoreDelete(_mutex);
  //   }
  // }
  
  // Getter for BotID (read-only)
    const char* getBotID() const {
        return _botID;
    }

  void setBotID(const char* botID) {
    strncpy(_botID, botID, sizeof(_botID) - 1);  // Copy safely to avoid overflow
    _botID[sizeof(_botID) - 1] = '\0';          // Ensure null-terminated string
  }

  //////////////////////////////////////
  // CURRENT GPS: Setters and Getters //
  //////////////////////////////////////
  void readGPS_mock(long &latitude_reading, long &longitude_reading) {

    // write mock data
    latitude_reading = 111; 
    longitude_reading = 222; 
  }

  void readGPS(long &latitude_reading, long &longitude_reading) {
    // Pull in all waiting bytes first
    while (SerialGPS.available()) {
      gps.encode(SerialGPS.read());
    }

    // Now check if we have a new valid location
    if (gps.location.isValid()) {
      long lat_data = lround(gps.location.lat() * 1e6);
      long lng_data = lround(gps.location.lng() * 1e6);

      latitude_reading = lat_data;
      longitude_reading = lng_data;

    } else {
      Serial.println("INVALID");
    }
  }

  void setCurrentGPS(long latitude, long longitude) {
    if(xSemaphoreTake(_xMutexSensor_Current_GPS, portMAX_DELAY) == pdTRUE) {
      _currentGPSLatitude = latitude;
      _currentGPSLongitude = longitude;
      xSemaphoreGive(_xMutexSensor_Current_GPS);
    }
  }
  
  void getCurrentGPS(long &latitude, long &longitude) {
    if(xSemaphoreTake(_xMutexSensor_Current_GPS, portMAX_DELAY) == pdTRUE) {
      latitude = _currentGPSLatitude;
      longitude = _currentGPSLongitude;
      xSemaphoreGive(_xMutexSensor_Current_GPS);
    }
  }
  

  /////////////////////////////////////
  // TARGET GPS: Setters and Getters //
  /////////////////////////////////////
  void setTargetGPS(long latitude, long longitude) {
    if(xSemaphoreTake(_xMutexSensor_Target_GPS, portMAX_DELAY) == pdTRUE) {
      _targetGPSLatitude = latitude;
      _targetGPSLongitude = longitude;
      xSemaphoreGive(_xMutexSensor_Target_GPS);
    }
  }
  
  void getTargetGPS(long &latitude, long &longitude) {
    if(xSemaphoreTake(_xMutexSensor_Target_GPS, portMAX_DELAY) == pdTRUE) {
      latitude = _targetGPSLatitude;
      longitude = _targetGPSLongitude;
      xSemaphoreGive(_xMutexSensor_Target_GPS);
    }
  }

  String getGPS_String() {
    long currentLon;
    long currentLat;

    getCurrentGPS(currentLon, currentLat);
    String gpsString = String(currentLat) + "," + String(currentLon);

    return gpsString;  // Return the formatted string
  }
  

  //////////////////////////////////////////////////////
  // IMU: Setters and Getters for the Compass Heading //
  //////////////////////////////////////////////////////
  float readIMU_mock() {
    float heading;

    if(xSemaphoreTake(_xMutexSensor_IMU, portMAX_DELAY) == pdTRUE) {
      heading = 2.2; // change to be real
      xSemaphoreGive(_xMutexSensor_IMU);
    }
    return heading;
  }

  void setHeading(float heading) {
    if(xSemaphoreTake(_xMutexSensor_IMU, portMAX_DELAY) == pdTRUE) {
      _compassHeading = heading;
      xSemaphoreGive(_xMutexSensor_IMU);
    }
  }
  
  float getHeading() {
    float heading = 0.0f;
    if(xSemaphoreTake(_xMutexSensor_IMU, portMAX_DELAY) == pdTRUE) {
      heading = _compassHeading;
      xSemaphoreGive(_xMutexSensor_IMU);
    }
    return heading;
  }
  /////////////////////////////////////////
  // Battery Sensor: Setters and Getters //
  /////////////////////////////////////////

  float readBatterySensor_mock() {
    float batteryLevel;
    if(xSemaphoreTake(_xMutexSensor_BatterySensor, portMAX_DELAY) == pdTRUE) {
      batteryLevel = 22.22; // change to be real
      xSemaphoreGive(_xMutexSensor_BatterySensor);
    }
    return batteryLevel;
  }

  void setBatteryLevel(float batteryLevel) {
    if(xSemaphoreTake(_xMutexSensor_BatterySensor, portMAX_DELAY) == pdTRUE) {
      _batteryLevel = batteryLevel;
      xSemaphoreGive(_xMutexSensor_BatterySensor);
    }
  }
  
  float getBatteryLevel() {
    float batteryLevel = 0.0f;
    if(xSemaphoreTake(_xMutexSensor_BatterySensor, portMAX_DELAY) == pdTRUE) {
      batteryLevel = _batteryLevel;
      xSemaphoreGive(_xMutexSensor_BatterySensor);
    }
    return batteryLevel;
  }

  ///////////////////////////
  // Change Automatic Mode //
  ///////////////////////////
  bool getAutomaticMode() {
    bool automaticMode;
    if(xSemaphoreTake(_xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
      automaticMode = _automaticMode;
      xSemaphoreGive(_xMutexAutomatic);
    }
    return automaticMode;
  }
  
  void setAutomaticMode(bool automaticMode) {
    if(xSemaphoreTake(_xMutexAutomatic, portMAX_DELAY) == pdTRUE) {
      _automaticMode = automaticMode;
      xSemaphoreGive(_xMutexAutomatic);
    }
  }

  ////////////////////////////////
  // Parsing and Format Methods //
  ////////////////////////////////
  String getAllData(){
    long currentGPSLatitude;
    long currentGPSLongitude;
    long targetGPSLatitude;
    long targetGPSLongitude;
    float compassHeading;
    float batteryLevel;

    // get target gps data
    getTargetGPS(targetGPSLatitude, targetGPSLongitude);

    // get current gps data
    getCurrentGPS(currentGPSLatitude, currentGPSLongitude);

    // get compass data
    compassHeading = getHeading();

    // get battery level data
    batteryLevel = getBatteryLevel();

    String data = "Target: " + String(targetGPSLatitude) + "," + String(targetGPSLongitude) +
               " | Current: " + String(currentGPSLatitude) + "," + String(currentGPSLongitude) +
               " | Heading: " + String(compassHeading) +
               " | Battery Level: " + String(batteryLevel);
  
    return data;
  }   

  // TD, Add battery sensor level
  String createPayload(){

    String latLon = getGPS_String();
    String createdString = "0," + String(_botID) + "," + latLon; // Why Zero? ohh zero is SBC

    return createdString;
  }

private:
  char _botID[20];

  // Sensor values
  long _currentGPSLatitude;
  long _currentGPSLongitude;
  float _compassHeading;
  long _targetGPSLatitude;
  long _targetGPSLongitude;
  float _batteryLevel;

  int _gps_period;
  int _imu_period;
  int _battery_sensor_period;

  bool _automaticMode = false;

  // Mutex to protect sensor data access
  SemaphoreHandle_t _xMutexSensor_IMU;
  SemaphoreHandle_t _xMutexSensor_Current_GPS;
  SemaphoreHandle_t _xMutexSensor_Target_GPS;
  SemaphoreHandle_t _xMutexSensor_BatterySensor;

  SemaphoreHandle_t _xMutexAutomatic;
};


class MotorController {
  public:
    Servo motor1;
    Servo motor2;

    int motorpin1 = 13;
    int motorpin2 = 2;

    int minUs = 1000;
    int maxUs = 2000;

    int frequency = 50;

    void setupMotors() {
      motor1.setPeriodHertz(frequency);// Standard 50hz servo
      motor2.setPeriodHertz(frequency);// Standard 50hz servo
      motor1.attach(motorpin1, minUs, maxUs); 
      motor2.attach(motorpin2, minUs, maxUs); 
    }

    void calibrateMotors(){
      // Calibrate Motors
      Serial.println("Calibrating Motors...");
      motor1.write(90);                 
      motor2.write(90);
      delay(5000); 
      Serial.println("Calibration Finished");
    }

    void move(int speed1, int speed2) {
        motor1.writeMicroseconds(speed1);
        motor2.writeMicroseconds(speed2);
    }

    void stopMotors(){
      motor1.write(90);                 
      motor2.write(90);
    }
};

// LoRa Start
// Set up the LoRa module for SX1278
SX1278 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);

String payload = "";
volatile bool receivedFlag = false;
volatile bool transmittedFlag = false;


SemaphoreHandle_t xSemaphore; // LoRa

MotorController motors;
BotData bot;

void setup() {
  // Start communication with the Serial Monitor (USB Serial)
  Serial.begin(115200); 
  delay(1000);

  // GPS Setup
  SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(3000);

  // Motor Setup
  motors.setupMotors();
  motors.calibrateMotors();

  // Setup Bot
  bot.setBotID(BOT_ID);


  setupLoRa();

  xSemaphore = xSemaphoreCreateBinary(); // mybe improve...
  xSemaphoreGive(xSemaphore); // mybe improve...

  xTaskCreate(BotTask, "Bot Task", 2048, NULL, 1, NULL);
  xTaskCreate(IMUTask, "IMU Task", 2048, NULL, 1, NULL);
  xTaskCreate(GPSTask, "GPS Task", 2048, NULL, 1, NULL);
  xTaskCreate(BSTask, "Battery Sensor Task", 2048, NULL, 1, NULL);

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
          payload = bot.createPayload();

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


// Automatic Task: Read Sensor Data and Move Bot
void AutomaticTask(void *pvParameters){
  while(1){
    if(bot.getAutomaticMode()){
      Serial.println("In Automatic Mode");

      String gpsString = ""; // Debug

      long targetLat; 
      long targetLon; 
      long currentLat;
      long currentLon;
      float currentCompass;

      // get current gps data
      bot.getCurrentGPS(currentLat, currentLon);

      // get target gps data
      bot.getTargetGPS(targetLat, targetLon);

      // get current imu data
      currentCompass = bot.getHeading();

      Serial.println(bot.getAllData()); // debug or can make a function that compiles a bunch of variables....
    }
    else{
      Serial.println("Not In Automatic Mode");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
  }
}

//////////////////
// Sensor Tasks //
//////////////////

// This will sample the GPS and IMU Sensors and write them to global variables 
void GPSTask(void *pvParameters){

  // These store local readings
  long local_GPS_Latitude;
  long local_GPS_Longitude; 

  while(1){
    bot.readGPS(local_GPS_Latitude, local_GPS_Longitude); // read the values, passed by refrence 
    bot.setCurrentGPS(local_GPS_Latitude, local_GPS_Longitude); // store them in global variables

    vTaskDelay(500 / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
  }
}

void IMUTask(void *pvParameters){

  // These store local readings
  float local_Compass_Heading; 

  while(1){
    local_Compass_Heading = bot.readIMU_mock(); // read the values, passed by refrence
    bot.setHeading(local_Compass_Heading); // store them in global variables
  
    vTaskDelay(IMU_Period / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
  }
}

void BSTask(void *pvParameters){

  // These store local readings
  float local_Battery_Level; 

  while(1){
    local_Battery_Level = bot.readBatterySensor_mock();
    bot.setBatteryLevel(local_Battery_Level); 

    vTaskDelay(Battery_Sensor_Period / portTICK_PERIOD_MS);  // Reduced from 200ms to 100ms
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

  if(BotID == bot.getBotID()){

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
      bot.setTargetGPS(latString.toInt(), lonString.toInt());

      bot.setAutomaticMode(true);
    } 

    if(Command == "AUTOSTOP"){
      bot.setAutomaticMode(false);
    }

    if(bot.getAutomaticMode()){
      return;
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

      motors.move(leftPWMString.toInt(), rightPWMString.toInt());

    }

    // STARTUP
    if(Command == "STARTUP"){
      // Serial.println("STARTUP");
      motors.stopMotors();
    }
    
    return true;
  } 
  else{
    Serial.println("Incorrect BotID"); // DEBUG
    return false;
  }
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



//////////////////////////
// GPS Helper Functions //
//////////////////////////

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










