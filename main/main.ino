// File to control all onboard software

// Move parsing funciton out of main file
// Add motor movement code

// Modes
// STARTUP
// MOV
// TEST
// MOVGPS
// MOVPWM
// INVALID
String STATE = "STARTUP";

void setup() {
  // Start communication with the Serial Monitor (USB Serial)
  Serial.begin(115200); // Standard baud rate for Serial Monitor
}

void loop() {

  String inputString = "";  // String to hold the incoming data

  // Read bytes from the serial until a newline character is found
  while (Serial.available() > 0) {

    // TODO: Change this to read from LoRA, or have the option to...
    char incomingByte = Serial.read();  // Read a byte from Serial

    if (incomingByte == '\n') {
      // When a newline character is received, print the input string

      // Print the message to the Serial Monitor as well
      Serial.print("Bot Received: ");
      Serial.println(inputString);  // This will print to the Serial Monitor

      // Split String
      int index = inputString.indexOf(' ');  // Find the position of the comma

      String inputKey = inputString.substring(0, index);      // "Hello"
      String OtherParts = inputString.substring(index + 1);    // "World"

      Serial.print("The Key is: ");
      Serial.println(inputKey);

      // Startup Mode: STARTUP
      if(inputKey == "STARTUP"){
        STATE = "STARTUP";
        Serial.println("In Startup Mode");
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
  }

  delay(1000); // 1-second delay
}

