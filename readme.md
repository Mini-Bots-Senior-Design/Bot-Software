

## Components

### LoRaDevice
Reads from I2C ports and writes the values to the uart port

### MotorController
Reads from the UART Port, moves the motor, and sends the messages via UDP


## Example Code
```
├── LoRaDevice                              
│   └── SensorRead
│       └── SensorRead.ino
│
├── MotorController                        
│   └── udp_server
        └── main
            └──udp_server.c
```