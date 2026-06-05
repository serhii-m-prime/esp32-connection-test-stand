#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "shared_types.h"


#if defined(ROLE_MASTER)
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
    #define CAN_TX_PIN   GPIO_NUM_11
    #define CAN_RX_PIN   GPIO_NUM_10
    #define I2C_SDA_PIN GPIO_NUM_5
    #define I2C_SCL_PIN GPIO_NUM_6

    #if defined(PROTOCOL_I2C)
        #define I2C_SDA_PIN GPIO_NUM_3
        #define I2C_SCL_PIN GPIO_NUM_4
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
    #define CAN_TX_PIN   GPIO_NUM_11
    #define CAN_RX_PIN   GPIO_NUM_10
    #define MOTION_SENSOR_PIN GPIO_NUM_5
    #define DISTANCE_SENSOR_PIN GPIO_NUM_6

    #if defined(PROTOCOL_I2C)
        #define I2C_SDA_PIN GPIO_NUM_3
        #define I2C_SCL_PIN GPIO_NUM_4
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
    while (1) {
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK) {
            if (message.identifier == 0x0F0) {
                ESP_LOGW("CAN_SERVICE", "Flashing command received via CAN!");
            }
        }
    }
}

void app_main() {

    init_permanent_can_service();
    xTaskCreate(can_background_rx_task, "can_bg_task", 3072, NULL, 10, NULL);

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