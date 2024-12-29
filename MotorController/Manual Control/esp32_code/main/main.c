/*
This program will run on an ESP32 to control the motors

Developed by Benji Gilbert and Noah Robitshek
*/

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/mcpwm_prelude.h"


#include "handlers.h"
#include "utils.h"
#include "globals.h"
#include "main.h"

static const char *TAG = "ESC_SETUP";

// Please consult the datasheet of your servo before changing the following parameters
#define ESC_MIN_PULSEWIDTH_US 1000  // Minimum pulse width in microsecond
#define ESC_MAX_PULSEWIDTH_US 2000  // Maximum pulse width in microsecond

#define ESC1_PULSE_GPIO             4
#define ESC2_PULSE_GPIO             15

#define ESC_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define ESC_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

bool motorInit = false;


// Init ESC 1
mcpwm_cmpr_handle_t esc1_comparator;
mcpwm_timer_handle_t esc1_timer;
mcpwm_oper_handle_t esc1_oper;
mcpwm_gen_handle_t esc1_generator;

// Init ESC 2
mcpwm_cmpr_handle_t esc2_comparator;
mcpwm_timer_handle_t esc2_timer;
mcpwm_oper_handle_t esc2_oper;
mcpwm_gen_handle_t esc2_generator;

// Create Timer 1
// ESP_LOGI(TAG, "Create timer and operator");
mcpwm_timer_config_t esc1_timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = ESC_TIMEBASE_RESOLUTION_HZ,
    .period_ticks = ESC_TIMEBASE_PERIOD,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
};

// Create Timer 2
// ESP_LOGI(TAG, "Create timer and operator");
mcpwm_timer_config_t esc2_timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = ESC_TIMEBASE_RESOLUTION_HZ,
    .period_ticks = ESC_TIMEBASE_PERIOD,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
};

mcpwm_operator_config_t esc1_operator_config = {
    .group_id = 0, // operator must be in the same group to the timer
};

mcpwm_operator_config_t esc2_operator_config = {
    .group_id = 0, // operator must be in the same group to the timer
};

mcpwm_comparator_config_t esc1_comparator_config = {
    .flags.update_cmp_on_tez = true,
};

mcpwm_comparator_config_t esc2_comparator_config = {
    .flags.update_cmp_on_tez = true,
};

mcpwm_generator_config_t esc1_generator_config = {
    .gen_gpio_num = ESC1_PULSE_GPIO,
};

mcpwm_generator_config_t esc2_generator_config = {
    .gen_gpio_num = ESC2_PULSE_GPIO,
};


void esc_init(){
    printf("esc_init start\n");
    ESP_ERROR_CHECK(mcpwm_new_timer(&esc1_timer_config, &esc1_timer));
    ESP_ERROR_CHECK(mcpwm_new_timer(&esc2_timer_config, &esc2_timer));


    ESP_ERROR_CHECK(mcpwm_new_operator(&esc1_operator_config, &esc1_oper));
    ESP_ERROR_CHECK(mcpwm_new_operator(&esc2_operator_config, &esc2_oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(esc1_oper, esc1_timer));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(esc2_oper, esc2_timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");

    ESP_ERROR_CHECK(mcpwm_new_comparator(esc1_oper, &esc1_comparator_config, &esc1_comparator));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");

    ESP_ERROR_CHECK(mcpwm_new_comparator(esc2_oper, &esc2_comparator_config, &esc2_comparator));


    ESP_ERROR_CHECK(mcpwm_new_generator(esc1_oper, &esc1_generator_config, &esc1_generator));


    ESP_ERROR_CHECK(mcpwm_new_generator(esc2_oper, &esc2_generator_config, &esc2_generator));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(esc1_generator,
                                                                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(esc1_generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, esc1_comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(esc2_generator,
                                                                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(esc2_generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, esc2_comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(esc1_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(esc1_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(esc2_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(esc2_timer, MCPWM_TIMER_START_NO_STOP));

    // printf("Start at half speed 1\n");

    printf("esc_init end\n");
}





// Task
// This task reads the bots direction and moves the bot.
static void state_machine(void *arg) { // maybe cast?
    while(1){
        switch (state) {
            case STATE_STARTUP:
                ESP_LOGI(TAG_STATE, "%s", "The Bot is in Startup Mode");

                break;

            case STATE_MOV:
                ESP_LOGI(TAG_STATE, "%s", "The Bot is in Manual Mode");

                break;

            case STATE_MOVGPS:
                ESP_LOGI(TAG_STATE, "%s", "The Bot is in Automatic Mode");

                break;

            case STATE_MOVPWM:
                ESP_LOGI(TAG_STATE, "%s", "The Bot is in PWM Mode");

                break;

            default:
                printf("Unknown state.\n");
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }  
}


void motor_init(){
    printf("Starting motor init\n");

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, 1500));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, 1500));
    
    // Delay for 10 seconds
    vTaskDelay(pdMS_TO_TICKS(10000)); 

    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, 1700));
    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, 1700));

    // vTaskDelay(pdMS_TO_TICKS(10000)); 


    motorInit = true;
    printf("Finishing motor init\n");
}


static void motor_control_task(void *arg){
    printf("Motor Task Starting\n");
    while(1){
        // TODO: Replace with reall commands
        printf("Left PWM: %d\n", leftPWM);
        printf("Right PWM: %d\n", rightPWM);
        
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, leftPWM));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, rightPWM));

        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for .1 second
    }
}


// Task
static void read_uart_task(void *arg)
{
    uart_init(); 

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {

        // Read Data from LoRa Device
        int len = uart_read_bytes(INPUT_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);

        // print user input
        if (len) {
            data[len] = '\0';
            
            parse_input_string((char *) data);
            // botDirection = (char *) data;
        }

        vTaskDelay(pdMS_TO_TICKS(10));     
    }
}


void app_main(void)
{
    // Enable or Disable Logging
    // esp_log_level_set(TAG_STATE, ESP_LOG_WARN);
    // esp_log_level_set(TAG_DIRECTION, ESP_LOG_WARN);
    // esp_log_level_set(TAG_STATE, ESP_LOG_WARN);

    esc_init();
    motor_init();

    // xTaskCreate(initialization_task, "initialization_task", TASK_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(read_uart_task, "read_uart_task", TASK_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(state_machine, "state_machine", TASK_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(motor_control_task, "motor_control_task", TASK_STACK_SIZE, NULL, 1, NULL);
}

