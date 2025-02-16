// File to control all onboard software

// Move parsing funciton out of main file
// Add motor movement code

// Modes
// STARTUP
// MOV
// TEST
// MOVGPS
// MOVPWM
String STATE = "STARTUP";

// Define PWM output pin
#define ESC1_PWM_PIN 13  // Pin for Motor 1
#define ESC2_PWM_PIN 2 // Pin for Motor 2

// Define pulse width range (in microseconds)
#define ESC_MIN_PULSEWIDTH_US 1000  // 1ms = Minimum speed (or stop)
#define ESC_MAX_PULSEWIDTH_US 2000  // 2ms = Maximum speed

// Define PWM frequency parameters
#define PWM_FREQUENCY_HZ 50   // 50Hz frequency (standard for ESCs/servos)
#define PWM_PERIOD_MS   20    // 20ms period (1/Frequency)

// Current pulse width (start at minimum)
int pulseWidth = 1500; // Middle Range 

int leftPulseWidth = 1500; // Middle Range 
int rightPulseWidth = 1500; // Middle Range 


void setup() {
  // Start communication with the Serial Monitor (USB Serial)
  Serial.begin(115200); // Standard baud rate for Serial Monitor

  pinMode(ESC1_PWM_PIN, OUTPUT);
  // pinMode(ESC2_PWM_PIN, OUTPUT);

  // Create tasks and pin them to Core 1
  xTaskCreatePinnedToCore(
    motor1Control,   // Function to run
    "Motor1 Task",   // Name of the task
    1000,            // Stack size (bytes)
    NULL,            // Task input parameter
    1,               // Task priority (1 is low)
    NULL,            // Task handle
    1                // Core to run on (Core 1)
  );

}

// Task to be run on Core 1
void loop() {
    String inputString = "";  // String to hold the incoming data

    // Read bytes from the serial until a newline character is found
    while (Serial.available() > 0) {

      // TODO: Change this to read from LoRA, or have the option to...
      char incomingByte = Serial.read();  // Read a byte from Serial

      if (incomingByte == '\n') {

        // Split String
        int index = inputString.indexOf(' ');  // Find the position of the comma

        String inputKey = inputString.substring(0, index);      // "Hello"
        String OtherParts = inputString.substring(index + 1);    // "World"

        // Startup Mode: STARTUP
        if(inputKey == "STARTUP"){
          STATE = "STARTUP";
          Serial.println("In Startup Mode");

          // TODO: add code to set pulseWidth to be 1500
          pulseWidth = 1500;


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
            pulseWidth = 1700;
          }
          else if(direction == "s"){
            Serial.println("Backward");
            pulseWidth = 1300;
          }
          else {
            Serial.println("Not sure what the input is??");
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

        // GPS Mode: MOVGPS

        // PWM Mode: MOVPWM
      
      // Clear the string for the next line
      inputString = "";
      } else {
        // Otherwise, add the byte to the string
        inputString += incomingByte;
      }

      delay(100);
    }
}


// currently working
void motor1Control(void *parameter) {
  while (true) {
    // Test
    digitalWrite(ESC1_PWM_PIN, HIGH);  // Start PWM pulse
    delayMicroseconds(pulseWidth);     // Wait for pulseWidth duration in milliseconds (non-blocking)
    digitalWrite(ESC1_PWM_PIN, LOW);  // Start PWM pulse
    delayMicroseconds(20000 - pulseWidth);     // Wait for pulseWidth duration in milliseconds (non-blocking)
  }
}



