#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "led_strip.h" // LED WS2812 driver
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

// Own includs
#include "shared_types.h"

#define CAN_ID_DIAG_ERR 0x0E0
#define CAN_ID_OTA_CMD  0x0F0
#define CAN_ID_OTA_DATA 0x0F1

#define INTERNAL_RGB_LED_GPIO  GPIO_NUM_21

#if defined(ROLE_MASTER)
    #define MY_NODE_ID 0

    #define CAN_TX_PIN   GPIO_NUM_2
    #define CAN_RX_PIN   GPIO_NUM_1
    #define DISPLAY_SCL_PIN GPIO_NUM_8
    #define DISPLAY_SDA_PIN GPIO_NUM_9
    #define DISPLAY_RES_PIN GPIO_NUM_10
    #define DISPLAY_DC_PIN GPIO_NUM_11
    #define DISPLAY_SC_PIN GPIO_NUM_12

    #if defined(PROTOCOL_I2C)
        #define I2C_SDA_PIN GPIO_NUM_3
        #define I2C_SCL_PIN GPIO_NUM_4
    #elif defined(PROTOCOL_I2S)
        #define I2S_WS_PIN GPIO_NUM_3
        #define I2S_BCLK_PIN GPIO_NUM_4
        #define I2S_DIN_PIN GPIO_NUM_6
    #elif defined(PROTOCOL_SPI)
        #define SPI_MOSI_PIN GPIO_NUM_4
        #define SPI_MISO_PIN GPIO_NUM_7
        #define SPI_SCLK_PIN GPIO_NUM_3
        #define SPI_SLAVE_1_PIN GPIO_NUM_13
        #define SPI_SLAVE_2_PIN GPIO_NUM_14
    #elif defined(PROTOCOL_UART)
        #define L1_UART_TX_PIN GPIO_NUM_3
        #define L1_UART_RX_PIN GPIO_NUM_4
        #define L2_UART_TX_PIN GPIO_NUM_5
        #define L2_UART_RX_PIN GPIO_NUM_6
    #endif


#elif defined(ROLE_SLAVE_ENV)
    #define MY_NODE_ID 1

    #define CAN_TX_PIN   GPIO_NUM_11
    #define CAN_RX_PIN   GPIO_NUM_10
    #define I2C_SDA_PIN GPIO_NUM_5
    #define I2C_SCL_PIN GPIO_NUM_6

    #if defined(PROTOCOL_I2C)
        #define I2C_SDA_PIN GPIO_NUM_3
        #define I2C_SCL_PIN GPIO_NUM_4
    #elif defined(PROTOCOL_I2S)
        #define I2S_WS_PIN GPIO_NUM_1
        #define I2S_BCLK_PIN GPIO_NUM_2
        #define I2S_DOUT_PIN GPIO_NUM_3
    #elif defined(PROTOCOL_SPI)
        #define SPI_MOSI_PIN GPIO_NUM_4
        #define SPI_MISO_PIN GPIO_NUM_7
        #define SPI_SCLK_PIN GPIO_NUM_3
        #define SPI_SLAVE_1_PIN GPIO_NUM_1
    #elif defined(PROTOCOL_UART)
        #define L1_UART_TX_PIN GPIO_NUM_3
        #define L1_UART_RX_PIN GPIO_NUM_4
    #endif

#elif defined(ROLE_SLAVE_MOTION)
    #define MY_NODE_ID 2

    #define CAN_TX_PIN   GPIO_NUM_11
    #define CAN_RX_PIN   GPIO_NUM_10
    #define MOTION_SENSOR_PIN GPIO_NUM_5
    #define DISTANCE_SENSOR_PIN GPIO_NUM_6

    #if defined(PROTOCOL_I2C)
        #define I2C_SDA_PIN GPIO_NUM_3
        #define I2C_SCL_PIN GPIO_NUM_4
    #elif defined(PROTOCOL_I2S)
        #define I2S_WS_PIN GPIO_NUM_1
        #define I2S_BCLK_PIN GPIO_NUM_2
        #define I2S_DOUT_PIN GPIO_NUM_3
    #elif defined(PROTOCOL_SPI)
        #define SPI_MOSI_PIN GPIO_NUM_4
        #define SPI_MISO_PIN GPIO_NUM_7
        #define SPI_SCLK_PIN GPIO_NUM_3
        #define SPI_SLAVE_2_PIN GPIO_NUM_1
    #elif defined(PROTOCOL_UART)
        #define L1_UART_TX_PIN GPIO_NUM_3
        #define L1_UART_RX_PIN GPIO_NUM_4
    #endif

#endif

static led_strip_handle_t led_strip;

void heartbeat_led_task(void *pvParameters) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = INTERNAL_RGB_LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
    };
    
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    uint8_t r = 0, g = 0, b = 0;

    #if defined(ROLE_MASTER)
        g = 15;
    #elif defined(ROLE_SLAVE_ENV)
        b = 20;
    #elif defined(ROLE_SLAVE_MOTION)
        r = 15; g = 15;
    #endif

    uint32_t loop_count = 0;
    while (1) {
        led_strip_set_pixel(led_strip, 0, r, g, b);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(500));

        loop_count++;
        if (loop_count % 10 == 0) {
            ESP_LOGI("STATUS", "Node ID %d alive. Heartbeat count: %ld", MY_NODE_ID, loop_count);
        }
    }
}

// init permanent CAN service for OTA/Telemetry, which is used by all roles
void init_permanent_can_service(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
    }
}

// Background task to process background CAN/OTA commands
void can_background_rx_task(void *pvParameters) {
    twai_message_t message;
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = NULL;
    bool ota_ongoing = false;

    while (1) {
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK) {
            #if defined(ROLE_MASTER)
            if (message.identifier == CAN_ID_DIAG_ERR) {
                diagnostic_packet_t rx_diag;
                memcpy(&rx_diag, message.data, sizeof(diagnostic_packet_t));
                ESP_LOGE("NET_DEBUG", "ALARM: Node %d reported Error 0x%04X", rx_diag.node_id, rx_diag.error_code);
            }
            #else
            if (message.identifier == CAN_ID_OTA_CMD) {
                uint8_t target_node = message.data[0];
                uint8_t cmd = message.data[1]; 

                if (target_node == MY_NODE_ID) {
                    if (cmd == 1 && !ota_ongoing) {
                        update_partition = esp_ota_get_next_update_partition(NULL);
                        if (update_partition != NULL) {
                            if (esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle) == ESP_OK) {
                                ota_ongoing = true;
                            }
                        }
                    } 
                    else if (cmd == 2 && ota_ongoing) {
                        ota_ongoing = false;
                        if (esp_ota_end(ota_handle) == ESP_OK && esp_ota_set_boot_partition(update_partition) == ESP_OK) {
                            vTaskDelay(pdMS_TO_TICKS(500));
                            esp_restart();
                        }
                    }
                }
            } 
            else if (message.identifier == CAN_ID_OTA_DATA && ota_ongoing) {
                esp_ota_write(ota_handle, message.data, message.data_length_code);
            }
            #endif
        }
    }
}

#if defined(ROLE_MASTER)
void master_uart_to_can_task(void *pvParameters) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    
    // CRITICAL: Remap standard stdout/stdin to the newly installed UART driver
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    uint8_t body[13]; 
    uint8_t byte;

    while (1) {
        if (uart_read_bytes(UART_NUM_0, &byte, 1, portMAX_DELAY) > 0) {
            if (byte == 0xAA) {
                if (uart_read_bytes(UART_NUM_0, &byte, 1, portMAX_DELAY) > 0 && byte == 0x55) {
                    int collected = 0;
                    while (collected < 13) {
                        int res = uart_read_bytes(UART_NUM_0, &body[collected], 13 - collected, portMAX_DELAY);
                        if (res > 0) {
                            collected += res;
                        }
                    }

                    twai_message_t msg = {0};
                    msg.identifier = ((uint32_t)body[0] << 24) | ((uint32_t)body[1] << 16) | 
                                     ((uint32_t)body[2] << 8)  | body[3];
                    msg.data_length_code = body[4];
                    memcpy(msg.data, &body[5], 8);
                    
                    twai_transmit(&msg, portMAX_DELAY);
                }
            }
        }
    }
}
#endif

void app_main() {
    // Hold briefly to guarantee proper USB CDC channel stabilization on host OS
    vTaskDelay(pdMS_TO_TICKS(200));

    xTaskCreate(heartbeat_led_task, "heartbeat_task", 3072, NULL, 4, NULL);
    init_permanent_can_service();

    xTaskCreate(can_background_rx_task, "can_bg_task", 4096, NULL, 5, NULL);

    #if defined(ROLE_MASTER)
        ESP_LOGI("MAIN", "Initializing Master Node (UART0-to-CAN Bridge mode)...");
        xTaskCreate(master_uart_to_can_task, "master_uart_task", 4096, NULL, 5, NULL);
    #else
        ESP_LOGI("MAIN", "Initializing Slave Node (ID: %d)...", MY_NODE_ID);
    #endif

    // Define nodes comunication protocol
    #if defined(PROTOCOL_CAN)
        ESP_LOGI("MAIN", "Initializing inter-board CAN test layout...");
    #elif defined(PROTOCOL_I2C)
        ESP_LOGI("MAIN", "Initializing alternative inter-board I2C test layout...");
    #elif defined(PROTOCOL_I2S)
        ESP_LOGI("MAIN", "Initializing alternative inter-board I2S test layout...");
    #elif defined(PROTOCOL_SPI)
        ESP_LOGI("MAIN", "Initializing alternative inter-board SPI test layout...");
    #elif defined(PROTOCOL_UART)
        ESP_LOGI("MAIN", "Initializing alternative inter-board Dual UART test layout...");
    #endif
}