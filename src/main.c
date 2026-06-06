#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <unistd.h>
#include "shared_types.h"
#include "driver/usb_serial_jtag.h"

#define CAN_ID_OTA_CMD  0x0F0
#define CAN_ID_OTA_DATA 0x0F1

#define HEARTBEAT_LED_PIN  GPIO_NUM_21

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

void heartbeat_led_task(void *pvParameters) {
    gpio_reset_pin(HEARTBEAT_LED_PIN);
    gpio_set_direction(HEARTBEAT_LED_PIN, GPIO_MODE_OUTPUT);
    
    bool led_state = false;

    while (1) {
        gpio_set_level(HEARTBEAT_LED_PIN, led_state);
        led_state = !led_state;

        #if defined(ROLE_MASTER)
        char diagnostic_msg[] = "[DIRECT USB] Master node is alive and ticking...\r\n";
        usb_serial_jtag_write_bytes((uint8_t*)diagnostic_msg, sizeof(diagnostic_msg) - 1, 10 / portTICK_PERIOD_MS);
        #endif

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// init permanent CAN service for OTA/Telemetry, which is used by all roles
void init_permanent_can_service(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500 kbps for OTA/Telemetry
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        ESP_LOGI("CAN_SERVICE", "Background CAN OTA service permanently active.");
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
            
            #if !defined(ROLE_MASTER)
            
            if (message.identifier == CAN_ID_OTA_CMD) {
                uint8_t target_node = message.data[0];
                uint8_t cmd = message.data[1]; // 1 = Start, 2 = End

                if (target_node == MY_NODE_ID) {
                    if (cmd == 1 && !ota_ongoing) {
                        ESP_LOGI("OTA", "Starting OTA Update Process...");
                        update_partition = esp_ota_get_next_update_partition(NULL);
                        
                        if (update_partition != NULL) {
                            esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
                            if (err == ESP_OK) {
                                ota_ongoing = true;
                                ESP_LOGI("OTA", "OTA initialized. Accepting binary frames...");
                            } else {
                                ESP_LOGE("OTA", "esp_ota_begin failed! Error: %d", err);
                            }
                        }
                    } 
                    else if (cmd == 2 && ota_ongoing) {
                        ESP_LOGI("OTA", "Finalizing OTA Process...");
                        ota_ongoing = false;
                        
                        if (esp_ota_end(ota_handle) == ESP_OK) {
                            if (esp_ota_set_boot_partition(update_partition) == ESP_OK) {
                                ESP_LOGI("OTA", "Success! Rebooting into new firmware...");
                                vTaskDelay(pdMS_TO_TICKS(500));
                                esp_restart(); // Restart device to apply the new firmware
                            } else {
                                ESP_LOGE("OTA", "Failed to switch boot partition!");
                            }
                        } else {
                            ESP_LOGE("OTA", "esp_ota_end failed!");
                        }
                    }
                }
            } 
            else if (message.identifier == CAN_ID_OTA_DATA && ota_ongoing) {
                esp_err_t err = esp_ota_write(ota_handle, message.data, message.data_length_code);
                if (err != ESP_OK) {
                    ESP_LOGE("OTA", "Partition write failed!");
                }
            }
            
            #endif
        }
    }
}

#if defined(ROLE_MASTER)
// Task for Master Node to bridge Native USB-C data directly to CAN Bus
void master_usb_to_can_task(void *pvParameters) {
    uint8_t header[2];
    uint8_t body[13]; 

    while (1) {
        int read_len = usb_serial_jtag_read_bytes(&header[0], 1, pdMS_TO_TICKS(10));
        
        if (read_len > 0 && header[0] == 0xAA) {
            int read_len2 = usb_serial_jtag_read_bytes(&header[1], 1, pdMS_TO_TICKS(50));
            
            if (read_len2 > 0 && header[1] == 0x55) {
                int bytes_read = 0;
                bool timeout_occurred = false;
                
                while (bytes_read < 13) {
                    int res = usb_serial_jtag_read_bytes(body + bytes_read, 13 - bytes_read, pdMS_TO_TICKS(50));
                    if (res > 0) {
                        bytes_read += res;
                    } else {
                        timeout_occurred = true;
                        break;
                    }
                }

                if (!timeout_occurred) {
                    twai_message_t msg = {0};
                    msg.identifier = ((uint32_t)body[0] << 24) | ((uint32_t)body[1] << 16) | 
                                     ((uint32_t)body[2] << 8)  | body[3];
                    msg.data_length_code = body[4];
                    
                    for (int i = 0; i < msg.data_length_code && i < 8; i++) {
                        msg.data[i] = body[5 + i];
                    }
                    twai_transmit(&msg, portMAX_DELAY);
                }
            }
        }
    }
}
#endif

void app_main() {
    xTaskCreate(heartbeat_led_task, "heartbeat_task", 2048, NULL, 4, NULL);
    init_permanent_can_service();
    // xTaskCreate(can_background_rx_task, "can_bg_task", 4096, NULL, 5, NULL);

    #if defined(ROLE_MASTER)
        ESP_LOGI("MAIN", "Initializing Master Node (USB-to-CAN Bridge mode)...");
        xTaskCreate(master_usb_to_can_task, "master_usb_task", 3072, NULL, 12, NULL);
    #else
        ESP_LOGI("MAIN", "Initializing Slave Node (ID: %d)...", MY_NODE_ID);
    #endif

    // Define nodes comunication protocol
    #if defined(PROTOCOL_CAN)
        ESP_LOGI("MAIN", "Initializing inter-board CAN test layout...");
    #elif defined(PROTOCOL_I2C)
        ESP_LOGI("MAIN", "Initializing alternative inter-board I2C test layout...");
    #elif defined(PROTOCOL_SPI)
        ESP_LOGI("MAIN", "Initializing alternative inter-board SPI test layout...");
    #elif defined(PROTOCOL_UART)
        ESP_LOGI("MAIN", "Initializing alternative inter-board Dual UART test layout...");
    #endif
}