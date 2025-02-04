#define TX_PIN 1 // UART TX Pin
#define RX_PIN 3 // UART RX Pin

HardwareSerial OutputPort(2); // Use UART2

void setup() {
  // Start communication with UART2 using custom pins
  OutputPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Start communication with the Serial Monitor (USB Serial)
  Serial.begin(115200); // Standard baud rate for Serial Monitor
}

void loop() {

  String inputString = "";  // String to hold the incoming data

  // Read bytes from the serial until a newline character is found
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();  // Read a byte from Serial

    if (incomingByte == '\n') {
      // When a newline character is received, print the input string

      // Send the message via UART2 (using outputPort)
      OutputPort.println(inputString);

      // Print the message to the Serial Monitor as well
      Serial.print("Received: ");
      Serial.println(inputString);  // This will print to the Serial Monitor
      
      // Clear the string for the next line
      inputString = "";
    } else {
      // Otherwise, add the byte to the string
      inputString += incomingByte;
    }
  }

  delay(1000); // 1-second delay
}