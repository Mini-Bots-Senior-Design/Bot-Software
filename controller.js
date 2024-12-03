const dgram = require('dgram');
const readline = require('readline');

// UDP client setup
const client = dgram.createSocket('udp4');
const REMOTE_HOST = '172.20.10.2'; // Replace with your target IP address
const REMOTE_PORT = 8083;       // Replace with your target port number

// Readline interface for keyboard input
const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout
});

console.log('Control the bot using WASD keys. Press P to stop.');

// Function to send a UDP message
function sendUdpMessage(message) {
  const buffer = Buffer.from(message);
  client.send(buffer, REMOTE_PORT, REMOTE_HOST, (err) => {
    if (err) {
      console.error('Error sending UDP message:', err.message);
    } else {
      console.log(`Message sent: ${message}`);
    }
  });
}

// Handle keyboard input
rl.on('line', (input) => {
  const command = input.trim().toUpperCase();

  switch (command) {
    case 'W':
      sendUdpMessage('MOV,F');
      break;
    case 'A':
      sendUdpMessage('MOV, L');
      break;
    case 'S':
      sendUdpMessage('MOV, B');
      break;
    case 'D':
      sendUdpMessage('MOV, R');
      break;
    case 'P':
        sendUdpMessage('MOV, S');
        break;
    default:
      console.log('Invalid input. Use W, A, S, D to control and P to stop.');
  }
});