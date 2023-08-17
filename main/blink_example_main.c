/* Blink Example

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "led_strip.h"
#include "sdkconfig.h"

#include <bmp280.h>
#include <ds18x20.h>

#include <esp_err.h>
#include <esp_system.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"
#include "protocol_examples_common.h"

#include "cJSON.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
 or you can edit the following line and set a number here.
 */
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define RELAY_GPIO 5

#define CONFIG_EXAMPLE_I2C_MASTER_SDA 21
#define CONFIG_EXAMPLE_I2C_MASTER_SCL 22

static uint8_t s_led_state = 0;
static void blink_led(void) {
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void) {
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void ConfigureRelay(void) {
    gpio_reset_pin(RELAY_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO, 1);
}

static esp_mqtt_client_handle_t client;

void TestBmp280(void *pvParameter) {
    ESP_ERROR_CHECK(i2cdev_init());

    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    float pressure, temperature, humidity;

    while (1) {

        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK) {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        if (bme280p) {
            printf(", Humidity: %.2f\n", humidity);
        } else {
            printf("\n");
        }

        char* status = "online";
        esp_mqtt_client_publish(client, "shulginvn/esp32-test-01/status", status, strlen(status), 1, 0);

        char tempString[20];
        memset(tempString, 0x00, sizeof(tempString));
        sprintf(tempString, "%.1f", temperature);
        esp_mqtt_client_publish(client, "shulginvn/esp32-test-01/sensor/temperature/state", tempString, strlen(tempString), 1, 0);

        char pressureString[20];
        memset(pressureString, 0x00, sizeof(pressureString));
        sprintf(pressureString, "%.1f", pressure);
        esp_mqtt_client_publish(client, "shulginvn/esp32-test-01/sensor/pressure/state", pressureString, strlen(pressureString), 1, 0);

        char humidityString[20];
        memset(humidityString, 0x00, sizeof(humidityString));
        sprintf(humidityString, "%.1f", humidity);
        esp_mqtt_client_publish(client, "shulginvn/esp32-test-01/sensor/humidity/state", humidityString, strlen(humidityString), 1, 0);

        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}

//static const gpio_num_t SENSOR_GPIO = 4;
//static const char *TAG1 = "ds18x20_test";
//void TestDs18B20(void *pvParameter)
//{
// Make sure that the internal pull-up resistor is enabled on the GPIO pin
// so that one can connect up a sensor without needing an external pull-up.
// (Note: The internal (~47k) pull-ups of the ESP do appear to work, at
// least for simple setups (one or two sensors connected with short leads),
// but do not technically meet the pull-up requirements from the ds18x20
// datasheet and may not always be reliable. For a real application, a proper
// 4.7k external pull-up resistor is recommended instead!)
// gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);

//    ds18x20_addr_t addr;
//    size_t found;
//    ds18x20_scan_devices(SENSOR_GPIO, &addr, 1, &found);
//    ESP_LOGE(TAG1, "Founded %d with addr=%08x%08x:", found, (uint32_t)(addr >> 32), (uint32_t)addr);

//    float temperature;
//    esp_err_t res;
//    while (1)
//    {

//        res = ds18x20_measure_and_read(SENSOR_GPIO, addr, &temperature);
//
//        char s[20];
//        memset(s, 0x00, sizeof(s));
//        sprintf(s, "%.1f", temperature);
//    	esp_mqtt_client_publish(client, "homeassistant/sensor/temperature/state", s, 0, 1, 0);
//        if (res != ESP_OK)
//            ESP_LOGE(TAG1, "Could not read from sensor %08x%08x: %d (%s)",
//                    (uint32_t)(addr >> 32), (uint32_t)addr, res, esp_err_to_name(res));
//        else
//            ESP_LOGI(TAG1, "Sensor %08x%08x: %.2f°C",
//                    (uint32_t)(addr >> 32), (uint32_t)addr, temperature);

//        vTaskDelay(pdMS_TO_TICKS(1000));
//    }
//}

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    char *discoverAnswer0 =
            "{ \"name\": \"BMP280_temperature\", \"state_topic\": \"shulginvn/esp32-test-01/sensor/temperature/state\", \"availability_topic\": \"shulginvn/esp32-test-01/status\", \"unique_id\": \"ESPsensoresp_temperature_test\", \"device\": { \"identifiers\": \"782184353f68\", \"name\": \"esp32-test-01\", \"sw_version\": \"1.0\", \"model\": \"esp32dev\", \"manufacturer\": \"espressif\" }, \"device_class\": \"temperature\", \"unit_of_measurement\": \"°C\", \"state_class\": \"measurement\", \"platform\": \"mqtt\" }";
    char *discoverAnswer1 =
            "{ \"name\": \"BMP280_pressure\", \"state_topic\": \"shulginvn/esp32-test-01/sensor/pressure/state\", \"availability_topic\": \"shulginvn/esp32-test-01/status\", \"unique_id\": \"dallas-ESPsensoresp_pressure_test\", \"device\": { \"identifiers\": \"782184353f68\", \"name\": \"esp32-test-01\", \"sw_version\": \"1.0\", \"model\": \"esp32dev\", \"manufacturer\": \"espressif\" }, \"device_class\": \"pressure\", \"unit_of_measurement\": \"kPa\", \"state_class\": \"measurement\", \"platform\": \"mqtt\" }";
    char *discoverAnswer2 =
            "{ \"name\": \"BMP280_humidity\", \"state_topic\": \"shulginvn/esp32-test-01/sensor/humidity/state\", \"availability_topic\": \"shulginvn/esp32-test-01/status\", \"unique_id\": \"dallas-ESPsensoresp_humidity_test\", \"device\": { \"identifiers\": \"782184353f68\", \"name\": \"esp32-test-01\", \"sw_version\": \"1.0\", \"model\": \"esp32dev\", \"manufacturer\": \"espressif\" }, \"device_class\": \"humidity\", \"unit_of_measurement\": \"%\", \"state_class\": \"measurement\", \"platform\": \"mqtt\" }";
    char *discoverAnswer3 =
            "{ \"name\": \"Switch01\", \"command_topic\": \"shulginvn/esp32-test-01/switch1/set\", \"unique_id\": \"RelaySwitch1\", \"device\": { \"identifiers\": \"782184353f68\", \"name\": \"esp32-test-01\", \"sw_version\": \"1.0\", \"model\": \"esp32dev\", \"manufacturer\": \"espressif\" }, \"device_class\": \"switch\", \"platform\": \"mqtt\" }";

    // TODO need status online for sensors

    switch ((esp_mqtt_event_id_t) event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_subscribe(client, "shulginvn/esp32-test-01/switch1/set", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "homeassistant/status", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
//        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
//        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        if (strncmp(event->topic, "shulginvn/esp32-test-01/switch1/set", event->topic_len) == 0) {
            if (strncmp(event->data, "ON", event->data_len) == 0) {
                gpio_set_level(RELAY_GPIO, 0);
            } else if (strncmp(event->data, "OFF", event->data_len) == 0) {
                gpio_set_level(RELAY_GPIO, 1);
            }
        } else if (strncmp(event->topic, "homeassistant/status", event->topic_len) == 0) {
            if (strncmp(event->data, "online", event->data_len) == 0) {

                msg_id = esp_mqtt_client_publish(client, "homeassistant/sensor/esp32-test-01/temperature/config", discoverAnswer0, strlen(discoverAnswer0), 1, 0);
                ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

                msg_id = esp_mqtt_client_publish(client, "homeassistant/sensor/esp32-test-01/pressure/config", discoverAnswer1, strlen(discoverAnswer1), 1, 0);
                ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

                msg_id = esp_mqtt_client_publish(client, "homeassistant/sensor/esp32-test-01/humidity/config", discoverAnswer2, strlen(discoverAnswer2), 1, 0);
                ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

                msg_id = esp_mqtt_client_publish(client, "homeassistant/switch/esp32-test-01/switch1/config", discoverAnswer3, strlen(discoverAnswer3), 1, 0);
                ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            }
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = { .uri = "mqtt://192.168.1.102:1883", };
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void) {
    /* Configure the peripheral according to the LED type */
    configure_led();
    ConfigureRelay();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

    xTaskCreate(TestBmp280, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    while (1) {
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
