
#include "utils.h"

int forwardSpeed = 1700;
int reverseSpeed = 1300;

int turningMotorSpeed = 1600;
int stoppedSpeed = 1500;


void uart_init(){
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(INPUT_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(INPUT_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(INPUT_UART_PORT_NUM, INPUT_TXD, INPUT_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}



// Function to parse the input string
void parse_input_string(char *input) {
    
    printf("Parsing input string: %s\n", input);
    // printf("The logic statement with MOV is : %d\n", (strcmp(input, "MOV") == 0));
    // printf("The logic statement with STARTUP is : %d", (strcmp(input, "STARTUP") == 0));

    char *strippedInput = strtok(input, "\n");
    char *newState = strtok(strippedInput, " ");  // Tokenize the input by space

    printf("New State: %s\n", newState);  // Should print "MOV"


    if (strcmp(input, "STARTUP") == 0) {
        printf("STARTUP Mode\n");
        // change the state
        state = STATE_STARTUP;

        // maybe change the other variables
        // TODO
    } 
    
    else if (strcmp(input, "MOV") == 0) {
        // chance the state
        state = STATE_MOV;

        // set global variables
        char *botDirection = strtok(NULL, " ");  // Continue tokenizing
        printf("Bot Direction: %s\n", botDirection);

        handle_MOV(botDirection);
    } 

    else if (strcmp(input, "MOVGPS") == 0) {
        printf("GPS Mode\n");
        // change the state
        state = STATE_MOVGPS;

        // set the lat and lon
        // TODO
    } 

    else if (strcmp(input, "MOVPWM") == 0) {
        printf("MOVPWM Mode\n");
        // change the state
        state = STATE_MOVPWM;

        // set the left and right PWM
        leftPWM = atoi(strtok(NULL, " "));  // Continue tokenizing
        rightPWM = atoi(strtok(NULL, " "));  // Continue tokenizing

        printf("Parsing MOVPWM String");
        printf("rightPWM: %d\n", rightPWM);
        printf("leftPWM: %d\n", leftPWM);
        // TODO
    } 

    else {
        printf("Unknown command received: %s\n", input);
    } 
}

// Handle Functions
void handle_MOV(char *botDirection){
    printf("handling mov");
    
    switch (*botDirection) {
        case 'R':
            ESP_LOGI(TAG_DIRECTION, "%s", "Moving the Bot right");
            leftPWM = forwardSpeed;
            rightPWM = turningMotorSpeed;

            break;
        case 'L':
            ESP_LOGI(TAG_DIRECTION, "%s", "Moving the Bot left");
            leftPWM = turningMotorSpeed;
            rightPWM = forwardSpeed;

            break;
        case 'F':
            ESP_LOGI(TAG_DIRECTION, "%s", "Moving the Bot forward");
            leftPWM = forwardSpeed;
            rightPWM = forwardSpeed;

            break;
        case 'S':
            ESP_LOGI(TAG_DIRECTION, "%s", "Stopping the Bot");
            leftPWM = stoppedSpeed;
            rightPWM = stoppedSpeed;

            break;
        case 'B':
            ESP_LOGI(TAG_DIRECTION, "%s", "Reversing the Bot");
            leftPWM = reverseSpeed;
            rightPWM = reverseSpeed;

            break;
        default:
            printf("ERROR, Invalid Command\n");
            break;
    }
}
