#ifndef DEVICE_H
#define DEVICE_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h" 
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/temperature_sensor.h"
#include "driver/i2c.h"

#define MQ2_CHANNEL ADC_CHANNEL_6
#define MQ2_GPIO_PIN GPIO_NUM_34
#define GRAY_CHANNEL ADC_CHANNEL_7
#define GRAY_GPIO_PIN GPIO_NUM_35
#define RADAR_PIN GPIO_NUM_23
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_MAX_RETRIES 10

extern adc_oneshot_unit_handle_t adc_handle;
extern temperature_sensor_handle_t temp_sensor;

esp_err_t i2c_register_write_read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_register_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);

#endif