#ifndef BMP280_H
#define BMP280_H

#define BMP280_ADDRESS              0x76
#define BMP280_RESET_VALUE          0xB6
#define BMP280_CHIPID_REG           0xD0
#define BMP280_RESET_REG            0xE0
#define BMP280_CTRL_MEAS_REG        0xF4
#define BMP280_CONFIG_REG           0xF5
#define BMP280_PRESSURE_MSB_REG     0xF7
#define BMP280_PRESSURE_LSB_REG     0xF8
#define BMP280_PRESSURE_XLSB_REG    0xF9
#define BMP280_TEMPERATURE_MSB_REG  0xFA
#define BMP280_TEMPERATURE_LSB_REG  0xFB
#define BMP280_TEMPERATURE_XLSB_REG 0xFC

void bmp280_init(void);
void bmp280_read_data(float *temperature, float *pressure);

#endif