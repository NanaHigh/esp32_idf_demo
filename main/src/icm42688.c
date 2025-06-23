// Author: Chongbing Yu <bd8ejk@foxmail.com>
// Date: 2025-06-23

#include "device.h"
#include "icm42688.h"

static const char *TAG = "icm42688";

static esp_err_t icm42688_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    return i2c_register_write_byte(ICM42688_ADDRESS, reg_addr, data);
}

static esp_err_t icm42688_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_register_write_read(ICM42688_ADDRESS, reg_addr, data, len);
}

void icm42688_init(void)
{
    uint8_t id;
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_WHO_AM_I, &id, 1));
    ESP_LOGI(TAG, "ICM42688 WHO_AM_I = 0x%02X", id);

    if (id != 0x47) {
        ESP_LOGE(TAG, "ICM42688 not found!");
        return;
    }

    ESP_LOGI(TAG, "ICM42688 found at address 0x%02X", ICM42688_ADDRESS);

    // Reset ICM42688
    ESP_ERROR_CHECK(icm42688_register_write_byte(ICM42688_PWR_MGMT0, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Configure accelerometer and gyroscope
    ESP_ERROR_CHECK(icm42688_register_write_byte(ICM42688_ACCEL_CONFIG0, 0x50)); // 16G, 1000Hz
    ESP_ERROR_CHECK(icm42688_register_write_byte(ICM42688_GYRO_CONFIG0, 0x50));  // 2000DPS, 1000Hz

    // Enable accelerometer and gyroscope
    ESP_ERROR_CHECK(icm42688_register_write_byte(ICM42688_PWR_MGMT0, 0x0F));
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void icm42688_read_data(float* temp, float* acc, float* gyro)
{
    uint8_t temp_msb, temp_lsb;
    uint8_t acc_msb[3], acc_lsb[3];
    uint8_t gyro_msb[3], gyro_lsb[3];

    // Read temperature
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_TEMP_DATA1, &temp_msb, 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_TEMP_DATA0, &temp_lsb, 1));
    int16_t temp_raw = (temp_msb << 8) | temp_lsb;
    float temperature = (temp_raw / 132.48) + 25.0;
    ESP_LOGI(TAG, "Temperature: %.2f°C", temperature);
    temp[0] = temperature;

    // Read accelerometer data
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_ACCEL_DATA_X1, &acc_msb[0], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_ACCEL_DATA_X0, &acc_lsb[0], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_ACCEL_DATA_Y1, &acc_msb[1], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_ACCEL_DATA_Y0, &acc_lsb[1], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_ACCEL_DATA_Z1, &acc_msb[2], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_ACCEL_DATA_Z0, &acc_lsb[2], 1));

    float acc_x = ((int16_t)((acc_msb[0] << 8) | acc_lsb[0])) * 16000.0 / 32768.0;
    float acc_y = ((int16_t)((acc_msb[1] << 8) | acc_lsb[1])) * 16000.0 / 32768.0;
    float acc_z = ((int16_t)((acc_msb[2] << 8) | acc_lsb[2])) * 16000.0 / 32768.0;

    ESP_LOGI(TAG, "Accelerometer: X=%.2f, Y=%.2f, Z=%.2f", acc_x, acc_y, acc_z);
    acc[0] = acc_x;
    acc[1] = acc_y;
    acc[2] = acc_z;

    // Read gyroscope data
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_GYRO_DATA_X1, &gyro_msb[0], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_GYRO_DATA_X0, &gyro_lsb[0], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_GYRO_DATA_Y1, &gyro_msb[1], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_GYRO_DATA_Y0, &gyro_lsb[1], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_GYRO_DATA_Z1, &gyro_msb[2], 1));
    ESP_ERROR_CHECK(icm42688_register_read(ICM42688_GYRO_DATA_Z0, &gyro_lsb[2], 1));

    float gyro_x = ((int16_t)((gyro_msb[0] << 8) | gyro_lsb[0])) * 2000.0 / 32768.0;
    float gyro_y = ((int16_t)((gyro_msb[1] << 8) | gyro_lsb[1])) * 2000.0 / 32768.0;
    float gyro_z = ((int16_t)((gyro_msb[2] << 8) | gyro_lsb[2])) * 2000.0 / 32768.0;

    ESP_LOGI(TAG, "Gyroscope: X=%.2f, Y=%.2f, Z=%.2f", gyro_x, gyro_y, gyro_z);
    gyro[0] = gyro_x;
    gyro[1] = gyro_y;
    gyro[2] = gyro_z;
}
