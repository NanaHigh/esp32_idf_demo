// Author: Chongbing Yu <bd8ejk@foxmail.com>
// Date: 2025-06-23

#include "device.h"
#include "bh1750.h"

static const char *TAG = "bh1750";

static esp_err_t bh1750_register_write(uint8_t device_addr, uint8_t cmd)
{
    esp_err_t ret;
    for (int i = 0; i < I2C_MAX_RETRIES; i++) {
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, device_addr, &cmd, 1, 1000 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Retrying BH1750 write to device 0x%02X, command 0x%02X, attempt %d", device_addr, cmd, i + 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGE(TAG, "Failed to write to BH1750 device 0x%02X, command 0x%02X after %d attempts: %s", device_addr, cmd, I2C_MAX_RETRIES, esp_err_to_name(ret));
    return ret;
}

static esp_err_t bh1750_register_read(uint8_t device_addr, uint8_t cmd, uint8_t *data, size_t len)
{
    esp_err_t ret;
    for (int i = 0; i < I2C_MAX_RETRIES; i++) {
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, device_addr, &cmd, 1, data, len, 1000 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Retrying BH1750 read from device 0x%02X, command 0x%02X, attempt %d", device_addr, cmd, i + 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGE(TAG, "Failed to read from BH1750 device 0x%02X, command 0x%02X after %d attempts: %s", device_addr, cmd, I2C_MAX_RETRIES, esp_err_to_name(ret));
    return ret;
}

void bh1750_init(void)
{
    ESP_LOGI(TAG, "Initializing BH1750...");
    ESP_ERROR_CHECK(bh1750_register_write(BH1750_LOW_ADDR, BH1750_POWER_ON_CMD));
    ESP_ERROR_CHECK(bh1750_register_write(BH1750_LOW_ADDR, BH1750_RESET_CMD));
    ESP_LOGI(TAG, "BH1750 initialized successfully.");
}

float bh1750_read_light(void)
{
    uint8_t buf[2];
    ESP_ERROR_CHECK(bh1750_register_read(BH1750_LOW_ADDR, BH1750_CONTINUOUS_HIGH_RES_MODE_CMD, buf, sizeof(buf)));
    uint16_t raw_data = (buf[0] << 8) | buf[1];
    float lux = raw_data / 1.2;
    ESP_LOGI(TAG, "BH1750 Light Intensity: %.2f lux", lux);
    return lux;
}
