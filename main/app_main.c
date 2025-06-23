// Author: Chongbing Yu <bd8ejk@foxmail.com>
// Date: 2025-06-23

#include "device.h"
#include "esp_timer.h"
#include "icm42688.h"
#include "bh1750.h"
#include "bmp280.h"
#include "gray.h"
#include "mq2.h"
#include "radar.h"

adc_oneshot_unit_handle_t adc_handle = NULL;
static const char *TAG = "mqtt5_example";
static esp_mqtt_client_handle_t mqtt_client = NULL;

static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%" PRIi32, event_id); // Using PRIi32 to format int32_t type
        break;
    }
}

static esp_mqtt5_user_property_item_t user_property_arr[] = {
    {"board", "esp32"},
    {"u", "NanaHigh"},
    {"p", "123456"}
};

#define USE_PROPERTY_ARR_SIZE sizeof(user_property_arr) / sizeof(esp_mqtt5_user_property_item_t)

static void pin_init(void)
{
    // Initialize ADC unit
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1, // Use ADC unit 1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle)); // Initialize ADC unit

    // Configure MQ-2 sensor pin as analog input
    gpio_config_t mq2_io_conf = {
        .pin_bit_mask = (1ULL << MQ2_GPIO_PIN), // Configure GPIO 0
        .mode = GPIO_MODE_DISABLE, // Disable digital function, enable analog function
        .pull_up_en = GPIO_PULLUP_DISABLE, // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE, // Disable interrupt
    };
    gpio_config(&mq2_io_conf);

    // Configure gray sensor pin as analog input
    gpio_config_t gray_io_conf = {
        .pin_bit_mask = (1ULL << GRAY_GPIO_PIN), // Configure GPIO 3
        .mode = GPIO_MODE_DISABLE, // Disable digital function, enable analog function
        .pull_up_en = GPIO_PULLUP_DISABLE, // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE, // Disable interrupt
    };
    gpio_config(&gray_io_conf);

    // Configure radar pin as digital input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RADAR_PIN), // Configure GPIO4
        .mode = GPIO_MODE_INPUT, // Set as input mode
        .pull_up_en = GPIO_PULLUP_ENABLE, // Enable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE, // Disable interrupt
    };
    gpio_config(&io_conf);

    // Initialize I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER, // Set as master mode
        .sda_io_num = I2C_SDA_PIN, // Configure SDA pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Enable SDA pull-up
        .scl_io_num = I2C_SCL_PIN, // Configure SCL pin
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Enable SCL pull-up
        .master.clk_speed = 100000, // Set I2C clock speed to 100kHz
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_conf)); // Configure I2C parameters
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0)); // Install I2C driver
}

static adc_cali_handle_t cali_handle = NULL;

static void adc_calibration_init(void)
{
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));
    ESP_LOGI(TAG, "ADC calibration initialized.");
}

static void device_init(void)
{
    adc_calibration_init();
    icm42688_init();
    bh1750_init();
    bmp280_init();
}

// MQ2 task
void mq2_task(void *pvParameters)
{
    char *buf = malloc(256); // Dynamically allocate memory for buffer
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffer");
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        uint64_t timestamp = esp_timer_get_time() / 1000;
        uint32_t mq2_data = read_mq2_status();
        sprintf(buf, "{\"timestamp\":%llu,\"mq2_raw_data\":%ld}", timestamp, mq2_data);
        int msg_id = esp_mqtt_client_publish(mqtt_client, "/sensor/mq2", buf, 0, 0, 0);
        ESP_LOGI(TAG, "MQ2 JSON sent publish successful, msg_id=%d", msg_id);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Gray task
void gray_task(void *pvParameters)
{
    char *buf = malloc(256); // Dynamically allocate memory for buffer
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffer");
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        uint64_t timestamp = esp_timer_get_time() / 1000;
        uint32_t gray_data = read_gray_status();
        sprintf(buf, "{\"timestamp\":%llu,\"gray_raw_data\":%ld}", timestamp, gray_data);
        int msg_id = esp_mqtt_client_publish(mqtt_client, "/sensor/gray", buf, 0, 0, 0);
        ESP_LOGI(TAG, "Gray JSON sent publish successful, msg_id=%d", msg_id);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Radar task
void radar_task(void *pvParameters)
{
    char *buf = malloc(256); // Dynamically allocate memory for buffer
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffer");
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        uint64_t timestamp = esp_timer_get_time() / 1000;
        bool radar_status = read_radar_status();
        sprintf(buf, "{\"timestamp\":%llu,\"radar_status\":\"%s\"}", timestamp, radar_status ? "HIGH" : "LOW");
        int msg_id = esp_mqtt_client_publish(mqtt_client, "/sensor/radar", buf, 0, 0, 0);
        ESP_LOGI(TAG, "Radar JSON sent publish successful, msg_id=%d", msg_id);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ICM42688 task
void icm42688_task(void *pvParameters)
{
    char *buf = malloc(256); // Dynamically allocate memory for buffer
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffer");
        vTaskDelete(NULL);
        return;
    }
    float temp[1], acc[3], gyro[3];
    while (1) {
        uint64_t timestamp = esp_timer_get_time() / 1000;
        icm42688_read_data(temp, acc, gyro);
        sprintf(buf, "{\"timestamp\":%llu,\"temperature\":%.2f,\"acceleration\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},\"gyroscope\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}}",
                timestamp, temp[0], acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
        int msg_id = esp_mqtt_client_publish(mqtt_client, "/sensor/icm42688", buf, 0, 0, 0);
        ESP_LOGI(TAG, "ICM42688 JSON sent publish successful, msg_id=%d", msg_id);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// BH1750 task
void bh1750_task(void *pvParameters)
{
    char *buf = malloc(256); // Dynamically allocate memory for buffer
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffer");
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        uint64_t timestamp = esp_timer_get_time() / 1000;
        float lux = bh1750_read_light();
        sprintf(buf, "{\"timestamp\":%llu,\"light_intensity\":%.2f}", timestamp, lux);
        int msg_id = esp_mqtt_client_publish(mqtt_client, "/sensor/bh1750", buf, 0, 0, 0);
        ESP_LOGI(TAG, "BH1750 JSON sent publish successful, msg_id=%d", msg_id);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// BMP280 task
void bmp280_task(void *pvParameters)
{
    char *buf = malloc(256); // Dynamically allocate memory for buffer
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffer");
        vTaskDelete(NULL);
        return;
    }
    float bmp_temp, bmp_press;
    while (1) {
        uint64_t timestamp = esp_timer_get_time() / 1000;
        bmp280_read_data(&bmp_temp, &bmp_press);
        sprintf(buf, "{\"timestamp\":%llu,\"temperature\":%.2f,\"pressure\":%.2f}", timestamp, bmp_temp, bmp_press);
        int msg_id = esp_mqtt_client_publish(mqtt_client, "/sensor/bmp280", buf, 0, 0, 0);
        ESP_LOGI(TAG, "BMP280 JSON sent publish successful, msg_id=%d", msg_id);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void mqtt5_app_start(void)
{
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .payload_format_indicator = true,
        .message_expiry_interval = 10,
        .response_topic = "/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = true,
        .credentials.username = "123",
        .credentials.authentication.password = "456",
        .session.last_will.topic = "/topic/will",
        .session.last_will.msg = "i will leave",
        .session.last_will.msg_len = 12,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt5_cfg);

    /* Set connection properties and user properties */
    esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_connect_property(client, &connect_property);

    /* Delete user properties after setting them */
    esp_mqtt5_client_delete_user_property(connect_property.user_property);
    esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(client);

    mqtt_client = client; // Save MQTT client handle
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    pin_init();
    device_init();
    mqtt5_app_start();

    // Create tasks
    xTaskCreate(mq2_task, "mq2_task", 4096, NULL, 5, NULL);
    xTaskCreate(gray_task, "gray_task", 4096, NULL, 5, NULL);
    xTaskCreate(radar_task, "radar_task", 4096, NULL, 5, NULL);
    xTaskCreate(icm42688_task, "icm42688_task", 4096, NULL, 5, NULL);
    xTaskCreate(bh1750_task, "bh1750_task", 4096, NULL, 5, NULL);
    xTaskCreate(bmp280_task, "bmp280_task", 4096, NULL, 5, NULL);
}