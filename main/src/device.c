// Author: Chongbing Yu <bd8ejk@foxmail.com>
// Date: 2025-06-23

#include "device.h"

static const char *TAG = "device";

esp_err_t i2c_register_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    esp_err_t ret;
    for (int i = 0; i < I2C_MAX_RETRIES; i++) {
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, device_addr, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Retrying I2C write to device 0x%02X, register 0x%02X, attempt %d", device_addr, reg_addr, i + 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ESP_LOGE(TAG, "Failed to write to device 0x%02X, register 0x%02X after %d attempts: %s", device_addr, reg_addr, I2C_MAX_RETRIES, esp_err_to_name(ret));
    return ret;
}

esp_err_t i2c_register_write_read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t ret;
    for (int i = 0; i < I2C_MAX_RETRIES; i++) {
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, device_addr, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Retrying I2C read from device 0x%02X, register 0x%02X, attempt %d", device_addr, reg_addr, i + 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ESP_LOGE(TAG, "Failed to read from device 0x%02X, register 0x%02X after %d attempts: %s", device_addr, reg_addr, I2C_MAX_RETRIES, esp_err_to_name(ret));
    return ret;
}