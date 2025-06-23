// Author: Chongbing Yu <bd8ejk@foxmail.com>
// Date: 2025-06-23

#include "device.h"
#include "bmp280.h"

static const char *TAG = "bmp280";

static struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib;


static esp_err_t bmp280_register_write(uint8_t reg_addr, uint8_t data)
{
    return i2c_register_write_byte(BMP280_ADDRESS, reg_addr, data);
}

static esp_err_t bmp280_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_register_write_read(BMP280_ADDRESS, reg_addr, data, len);
}

static void bmp280_read_calibration_data(void)
{
    uint8_t calib_data[24];
    ESP_ERROR_CHECK(bmp280_register_read(0x88, calib_data, sizeof(calib_data)));

    bmp280_calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    bmp280_calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    bmp280_calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    bmp280_calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    bmp280_calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    bmp280_calib.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    bmp280_calib.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    bmp280_calib.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    bmp280_calib.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    bmp280_calib.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    bmp280_calib.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    bmp280_calib.dig_P9 = (calib_data[23] << 8) | calib_data[22];

    ESP_LOGI(TAG, "Calibration data read successfully.");
}

static float bmp280_compensate_temperature(int32_t adc_T, int32_t *t_fine)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) *
            ((int32_t)bmp280_calib.dig_T2)) >>
           11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >>
             12) *
            ((int32_t)bmp280_calib.dig_T3)) >>
           14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;
}

static float bmp280_compensate_pressure(int32_t adc_P, int32_t t_fine)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)bmp280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_calib.dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);
    return (float)p / 25600.0;
}

void bmp280_init(void)
{
    ESP_LOGI(TAG, "Initializing BMP280...");
    uint8_t chip_id;
    ESP_ERROR_CHECK(bmp280_register_read(BMP280_CHIPID_REG, &chip_id, 1));
    ESP_LOGI(TAG, "BMP280 Chip ID: 0x%02X", chip_id);

    if (chip_id != 0x58) {
        ESP_LOGE(TAG, "BMP280 not found!");
        return;
    }

    ESP_ERROR_CHECK(bmp280_register_write(BMP280_RESET_REG, BMP280_RESET_VALUE));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    bmp280_read_calibration_data(); // Read calibration parameters

    uint8_t ctrl_meas = 0x27; // Temperature and pressure measurement, normal mode
    uint8_t config = 0xA0;    // Configure filter and standby time
    ESP_ERROR_CHECK(bmp280_register_write(BMP280_CTRL_MEAS_REG, ctrl_meas));
    ESP_ERROR_CHECK(bmp280_register_write(BMP280_CONFIG_REG, config));
    ESP_LOGI(TAG, "BMP280 initialized successfully.");
}

void bmp280_read_data(float *temperature, float *pressure)
{
    uint8_t temp_msb, temp_lsb, temp_xlsb;
    uint8_t press_msb, press_lsb, press_xlsb;

    // Read temperature data
    ESP_ERROR_CHECK(bmp280_register_read(BMP280_TEMPERATURE_MSB_REG, &temp_msb, 1));
    ESP_ERROR_CHECK(bmp280_register_read(BMP280_TEMPERATURE_LSB_REG, &temp_lsb, 1));
    ESP_ERROR_CHECK(bmp280_register_read(BMP280_TEMPERATURE_XLSB_REG, &temp_xlsb, 1));
    int32_t temp_raw = ((temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4));

    // Read pressure data
    ESP_ERROR_CHECK(bmp280_register_read(BMP280_PRESSURE_MSB_REG, &press_msb, 1));
    ESP_ERROR_CHECK(bmp280_register_read(BMP280_PRESSURE_LSB_REG, &press_lsb, 1));
    ESP_ERROR_CHECK(bmp280_register_read(BMP280_PRESSURE_XLSB_REG, &press_xlsb, 1));
    int32_t press_raw = ((press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4));

    // Compensate data using calibration parameters
    int32_t t_fine;
    *temperature = bmp280_compensate_temperature(temp_raw, &t_fine) / 100.0; // Convert to Celsius
    *pressure = bmp280_compensate_pressure(press_raw, t_fine);               // Convert to hPa

    ESP_LOGI(TAG, "BMP280 Temperature: %.2f°C, Pressure: %.2f hPa", *temperature, *pressure);
}