/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include <string.h>
#include <sys/param.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "driver/i2c.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <math.h>
#include <driver/uart.h>

// for GPIO
#include "driver/gpio.h"

#define PORT CONFIG_EXAMPLE_PORT
#define SERVER_IP CONFIG_EXAMPLE_SERVER_IP
#define SERVER_PORT CONFIG_EXAMPLE_SERVER_PORT
static const char *TAG = "example";

// Please consult the datasheet of your servo before changing the following parameters
#define ESC_MIN_PULSEWIDTH_US 1000  // Minimum pulse width in microsecond
#define ESC_MAX_PULSEWIDTH_US 2000  // Maximum pulse width in microsecond

#define ESC1_PULSE_GPIO             4
#define ESC2_PULSE_GPIO             15
#define ESC_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define ESC_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define DEMO_MODE 1
#define TESTING_MODE 0


// UART Defs
#define UART_PORT UART_NUM_2      // Using UART1 for this example
#define UART_TX_PIN 17           // Define TX pin (if needed)
#define UART_RX_PIN 16           // Define RX pin
#define UART_BUF_SIZE (1024)     // UART buffer size


int leftPWM = 1500;
int rightPWM = 1500;

int factor = 5;

int buoyID = 1; // HARDCODED FOR NOW, BUT WILL BE SENT FROM SBC ON INITIAL CONNECTION

float GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, MagX, MagY, MagZ, LatCurr, LonCurr, LatTarget, LonTarget;

static void uart_init() {
    vTaskDelay(pdMS_TO_TICKS(10000));
    const uart_config_t uart_config = {
        .baud_rate = 115200,                      // Match with master
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART parameters
    uart_param_config(UART_PORT, &uart_config);

    // Set UART pins (you can set UART_PIN_NO_CHANGE for unused pins)
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver using the buffer size
    uart_driver_install(UART_PORT, UART_BUF_SIZE, 0, 0, NULL, 0);

    printf("UART initialized on port %d\n", UART_PORT);
}


// static void esc_init(mcpwm_timer_handle_t *timer, mcpwm_oper_handle_t *oper, mcpwm_cmpr_handle_t *comparator, mcpwm_gen_handle_t *generator, int gpio_num) {
//     ESP_LOGI(TAG, "Create timer and operator");
//     mcpwm_timer_config_t timer_config = {
//         .group_id = 0,
//         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//         .resolution_hz = ESC_TIMEBASE_RESOLUTION_HZ,
//         .period_ticks = ESC_TIMEBASE_PERIOD,
//         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

//     mcpwm_operator_config_t operator_config = {
//         .group_id = 0, // operator must be in the same group to the timer
//     };
//     ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

//     ESP_LOGI(TAG, "Connect timer and operator");
//     ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

//     ESP_LOGI(TAG, "Create comparator and generator from the operator");
//     mcpwm_comparator_config_t comparator_config = {
//         .flags.update_cmp_on_tez = true,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

//     mcpwm_generator_config_t generator_config = {
//         .gen_gpio_num = gpio_num,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

//     // set the initial compare value, so that the servo will spin to the center position
//     // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, ESC_MIN_PULSEWIDTH_US));

//     ESP_LOGI(TAG, "Set generator action on timer and compare event");
//     // go high on counter empty
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
//                                                               MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
//     // go low on compare threshold
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
//                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

//     ESP_LOGI(TAG, "Enable and start timer");
//     ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
//     ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

//     printf("Start at half speed\n");
//     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 1500));
//     vTaskDelay(10000 / portTICK_PERIOD_MS);

// }

int mock_uart_read_bytes(int uart_num, uint8_t* data, size_t length, int ticks_to_wait) {
    // Simulate 11 floats of data (44 bytes total)
    if (length != 44) return -1;

    float mockSensorData[11] = {
        1.1f, -2.2f, 3.3f,   // Accelerometer X, Y, Z
        4.4f, -5.5f, 6.6f,   // Gyroscope X, Y, Z
        7.7f, -8.8f, 9.9f,   // Magnetometer X, Y, Z
        37.7749f, -122.4194f // GPS Lat, Lon
    };

    memcpy(data, mockSensorData, length);
    return length;
}



static void uart_receive_task(void* pvParameters) {
    uint8_t data[44]; // Buffer for 11 floats (4 bytes each)

    while (1) {
        // int len = uart_read_bytes(UART_PORT, data, sizeof(data), pdMS_TO_TICKS(1000));
        int len = mock_uart_read_bytes(UART_PORT, data, sizeof(data), pdMS_TO_TICKS(1000));

        // printf("DATA: %s\n", data);
        if (len == sizeof(data)) {
            float* sensorData = (float*)data;

            AccelX = sensorData[0];
            AccelY = sensorData[1];
            AccelZ = sensorData[2];

            GyroX = sensorData[3];
            GyroY = sensorData[4];
            GyroZ = sensorData[5];

            MagX = sensorData[6];
            MagY = sensorData[7];
            MagZ = sensorData[8];

            LatCurr = sensorData[9];
            LonCurr = sensorData[10];

            printf("Received Data:\n");
            printf("Accelerometer (m/s^2) - X: %.2f, Y: %.2f, Z: %.2f\n", sensorData[0], sensorData[1], sensorData[2]);
            printf("Gyroscope (rad/s) - X: %.2f, Y: %.2f, Z: %.2f\n", sensorData[3], sensorData[4], sensorData[5]);
            printf("Magnetometer (uT) - X: %.2f, Y: %.2f, Z: %.2f\n", sensorData[6], sensorData[7], sensorData[8]);
            printf("GPS - Lat: %.6f, Lon: %.6f\n", sensorData[9], sensorData[10]);
            printf("--- End of Sensor Data Cycle ---\n");

        } else {
            printf("Data length mismatch. Received %d bytes.\n", len);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


static void udp_send_task() {
    char payload[256];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SERVER_PORT);

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", SERVER_IP, SERVER_PORT);

    while (1) {
        // Prepare the payload to send
        if (DEMO_MODE) {
            snprintf(payload, sizeof(payload), "DEMODATA,%i,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%i,%i", buoyID, LatCurr, LonCurr, GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, MagX, MagY, MagZ, leftPWM, rightPWM);
        }
        else if (!TESTING_MODE) {
            snprintf(payload, sizeof(payload), "POS,%i,%f,%f", buoyID, LatCurr, LonCurr);

        }
        else {
            snprintf(payload, sizeof(payload), "TESTDATA,%f,%f,%f,%f,%f,%f,%f,%f,%f,%i,%i", GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, MagX, MagY, MagZ, leftPWM, rightPWM);
        }
        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Message sent: %s", payload);

        // Wait for 5 seconds before sending the next message
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    if (sock != -1) {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
    vTaskDelete(NULL);
}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

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
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_config_t esc1_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = ESC_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = ESC_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&esc1_timer_config, &esc1_timer));

    // Create Timer 2
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_config_t esc2_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = ESC_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = ESC_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&esc2_timer_config, &esc2_timer));

    mcpwm_operator_config_t esc1_operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&esc1_operator_config, &esc1_oper));

    mcpwm_operator_config_t esc2_operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&esc2_operator_config, &esc2_oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(esc1_oper, esc1_timer));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(esc2_oper, esc2_timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t esc1_comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(esc1_oper, &esc1_comparator_config, &esc1_comparator));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t esc2_comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(esc2_oper, &esc2_comparator_config, &esc2_comparator));

    mcpwm_generator_config_t esc1_generator_config = {
        .gen_gpio_num = ESC1_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(esc1_oper, &esc1_generator_config, &esc1_generator));

    mcpwm_generator_config_t esc2_generator_config = {
        .gen_gpio_num = ESC2_PULSE_GPIO,
    };
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

    printf("Start at half speed 1\n");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, 1500));
    printf("Start at half speed 2\n");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, 1500));
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;
#endif
        while (1) {
            ESP_LOGI(TAG, "Waiting for data");
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
            int len = recvmsg(sock, &msg, 0);
#else
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
#endif
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
                    for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG, "dest ip: %s", inet_ntoa(pktinfo->ipi_addr));
                        }
                    }
#endif
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);


                // char str[] = "MOV, HR";

                // // Using strtok to split the string by comma and space
                char *token = strtok(rx_buffer, ", "); // Split by ", " (comma and space)


                // // Printing the Input String
                printf("%s\n", token);
                if (strcmp(token, "MOVGPS") == 0) {
                    token = strtok(NULL, ", "); // Move to the next token
                    LatTarget = atof(token);
                    printf("LATTARGET: %f\n", LatTarget);

                    token = strtok(NULL, ", ");
                    LonTarget = atof(token);
                    printf("LONTARGET: %f\n", LonTarget);
                }
                else if (strcmp(token, "MOV") == 0) {
                    token = strtok(NULL, ", "); // Move to the next token
                    if(strcmp(token, "R") == 0){
                        printf("Moving the Bot right\n");
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, 1800));
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, 2000));
                    }
                    else if(strcmp(token, "L") == 0){
                        printf("Moving the Bot left\n");
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, 1800));
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, 1500));
                    }
                    else if(strcmp(token, "F") == 0){
                        printf("Moving the Bot forward\n");
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, 1800));
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, 1800));
                    }
                    else if(strcmp(token, "S") == 0){
                        printf("Stopping the Bot\n");  
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, 1500));
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, 1500));
                    }
                    else if(strcmp(token, "B") == 0){
                        printf("Reverse the Bot\n");  
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, 1200 ));
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, 1200 ));
                    }
                    else{printf("ERROR, Invalid Command");}
                }
                else if (strcmp(token, "MOVPWM") == 0) {
                    token = strtok(NULL, ", ");
                    leftPWM = atoi(token);
                    printf("LEFT PWM %i\n", leftPWM);
                    token = strtok(NULL, ", ");
                    rightPWM = atoi(token);
                    printf("RIGHT PWM %i\n", rightPWM);
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc1_comparator, (int)leftPWM));
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc2_comparator, (int)rightPWM));

                }
                else {
                    printf("PARSER ERROR\n");
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    //ESP_ERROR_CHECK(i2c_slave_init());
    //ESP_LOGI(TAG, "I2C Slave initialized successfully");

    uart_init();

    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);

    // uncomment if you want to send send data
    // xTaskCreate(udp_send_task, "udp_send", 4096, NULL, 5, NULL);

    // Uncomment if you want to recieve data
    // xTaskCreate(uart_receive_task, "uart_receive_task", 4096, NULL, 5, NULL);

}