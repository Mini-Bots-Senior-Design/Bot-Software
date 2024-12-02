

## Components

#### 1. LoRaDevice
**Inputs**:
- N/A

**Actions**:
- Read from the GPS (i2c)
- Read from the IMU (i2c)

**Outputs**:
- Print to UART
  
#### 2. MotorController
**Inputs**:
- Read from UART Port
- Listen to UDP Messages

**Internals**:
- Move the Motors

**Outputs**:
- Publish data to Uart

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