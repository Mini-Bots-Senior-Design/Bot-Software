#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "globals.h"


#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define UART_BAUD_RATE 115200            // Baud rate for communication

#define INPUT_TXD 1
#define INPUT_RXD 3
#define INPUT_UART_PORT_NUM UART_NUM_0 

#define BUF_SIZE (1024)

// Function prototypes
void uart_init(void);  // UART initialization function

// Function to parse the input string
void parse_input_string(char *input);

void handle_MOV(char *botDirection);

#endif  // UART_COMM_H