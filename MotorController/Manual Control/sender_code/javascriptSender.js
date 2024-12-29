/*
This program will send commands to the bot via UART

-------------------------------------------
// Schema // 
MOV - Course movement
- MOV L: Turn Left
- MOV R: Turn Right
- MOV F: Go Straight
- MOV B: Go Backward
- MOV S: Stop

MOVPWM - Percise movement
- MOVPWM <leftPWM> <rightPWM>: Operate with Variable Speed Control

- UpArrow: Increase Both Motors PWM
- DownArrow: Decrease Both Motors PWM

- LeftArrow: Increase Right Motor, Decrease Left Motor
- RightArrow: Decrease Right Motor, Increase Left Motor

MOVGPS - GPS Movement
- MOVGPS <LatTarget> <LonTarget>: Move to a GPS Coordinate

STARTUP - Startup State
- STARTUP: Change to Startup State
-------------------------------------------


-------------------------------------------
// Keyboard Commands //
MOV - Course movement
a -> MOV L
d -> MOV R
w -> MOV F
s -> MOV B
p -> MOV S

MOVPWM - Percise movement
i -> MOVPWM <leftPWM> <rightPWM>

UpArrow -> MOVPWM <++leftPWM> <++rightPWM>
DownArrow -> MOVPWM <++leftPWM> <--rightPWM>
LeftArrow -> MOVPWM <--leftPWM> <++rightPWM>
DownArrow -> MOVPWM <--leftPWM> <--rightPWM>

MOVGPS - GPS Movement
g -> MOVGPS <LatTarget> <LonTarget>

STARTUP - Startup Setting
x -> STARTUP
-------------------------------------------


*/

const { SerialPort } = require('serialport');
const readline = require('readline');

// Replace with your serial port path and baud rate
const portPath = '/dev/cu.usbserial-0001'; // Adjust for your system
const baudRate = 115200;

// Enable raw mode to capture single keypresses
process.stdin.setRawMode(true);


var leftPWM = 1500;
var rightPWM = 1500;

var state = "STATE_STARTUP";

let buffer = ''; // Temporary buffer to hold incoming data
let flushed = false;


// Create a new serial port instance
const port = new SerialPort({
    path: portPath,
    baudRate: baudRate,
}, (err) => {
    if (err) {
        console.error('Error opening serial port:', err.message);
        process.exit(1);
    }
    console.log(`Serial port ${portPath} opened with baud rate ${baudRate}`);
});


// Set up readline interface for capturing keypresses
const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout,
    terminal: true,
});


///////////////////
// Console Setup //
///////////////////

console.log("////////////////////////////////////////")
console.log('Type Commands or Ctrl+C to exit.');

// Handle the "open" event
port.on('open', () => {
    console.log('Serial port is now open.');

    // Flush the port buffer initially
    port.flush((err) => {
        if (err) {
            console.error('Error flushing the port:', err.message);
            return;
        }
        console.log('Port buffer flushed.');
        console.log("////////////////////////////////////////")

        flushed = true;
    });
});

///////////////////
// Send Commands //
///////////////////

// Handle keypress events
rl.input.on('data', (data) => {
    const key = data.toString();

    if (key === '\u0003') { // Ctrl+C
        console.log('Exiting...');
        rl.close();
        process.exit();
    }

    let message = '';
    switch (key) {

        // MOV
        case 'w':
            state = "STATE_MOV";
            message = 'MOV F';
            break;
        case 'a':
            state = "STATE_MOV";
            message = 'MOV L';
            break;
        case 's':
            state = "STATE_MOV";
            message = 'MOV B';
            break;
        case 'd':
            state = "STATE_MOV";
            message = 'MOV R';
            break;
        case 'p':
            state = "STATE_MOV";
            message = 'MOV S';
            break;

        // MOVPWM
        case 'i':
            state = "STATE_MOVPWM";

            message = formatPWMString(leftPWM, rightPWM);
            break;

        // MOVGPS
        case 'g':
            state = "STATE_MOVGPS";
            message = 'MOVGPS 100 200';
            break;

        // STARTUP
        case 'x':
            state = "STATE_STARTUP";
            message = 'STARTUP';
            break;

        // Forward
        case '\u001b[A':
            if(state == "STATE_MOVPWM"){
                leftPWM = leftPWM + 50;
                rightPWM = rightPWM + 50;
    
                message = formatPWMString(leftPWM, rightPWM);
            }
            break;

        // Back
        case '\u001b[B':
            if(state == "STATE_MOVPWM"){
                leftPWM = leftPWM - 50;
                rightPWM = rightPWM - 50;

                message = formatPWMString(leftPWM, rightPWM);
            }
            break;
            

        // Right
        case '\u001b[C':
            if(state == "STATE_MOVPWM"){
                leftPWM = leftPWM + 50;
                rightPWM = rightPWM - 50;

                message = formatPWMString(leftPWM, rightPWM);
            }
            break;

        // Left
        case '\u001b[D':
            if(state == "STATE_MOVPWM"){
                leftPWM = leftPWM - 50;
                rightPWM = rightPWM + 50;

                message = formatPWMString(leftPWM, rightPWM);
            }
            break;

        default:
            console.error(`Error: Invalid key '${key}'`);
            return; // Do not send anything for invalid keys
        
    }

    // Send the message to the serial port
    if(message != ''){
        port.write(message + '\n', (err) => {
            if (err) {
                return console.error('Error writing to serial port:', err.message);
            }
            console.log(` -> Sent: ${message}`);
        });
    }
});


//////////////////
// Recieve Data //
//////////////////

// Process incoming data
port.on('data', (chunk) => {
    if(flushed){
        // Append incoming chunk to the buffer
        buffer += chunk.toString();

        // Split the buffer into lines based on newline character
        let lines = buffer.split('\n');

        // Process all lines except the last (which might be incomplete)
        lines.slice(0, -1).forEach((line) => {
            console.log(`Received: ${line.trim()}`);
        });

        // parse position data
        // format: "POS <buoyID> <LatCurr> <LonCurr> <Heading>"
        // format: "DEMODATA <buoyID> <LatCurr> <LonCurr> <Heading> <AccelX> <AccelY> <AccelZ>"
        if(line.trim().startsWith("POS")){

        }

        // Keep the last incomplete line in the buffer
        buffer = lines[lines.length - 1];
    }
});

function formatPWMString(leftPWM, rightPWM){
    return "MOVPWM " + leftPWM.toString() + " " + rightPWM.toString()
}