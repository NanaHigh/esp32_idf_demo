#include "device.h"
#include "gray.h"

static const char *TAG = "gray";

uint32_t read_gray_status()
{
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, GRAY_CHANNEL, &chan_cfg));

    int raw_value = 0;
    int valid_samples = 10; // Number of valid samples
    int discard_samples = 10; // Number of discarded samples

    // Discard the first 'discard_samples' readings
    for (int i = 0; i < discard_samples; i++) {
        int temp_value;
        esp_err_t ret = adc_oneshot_read(adc_handle, GRAY_CHANNEL, &temp_value);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ADC read timeout during discard phase");
            continue; // Skip this reading
        }
        ESP_ERROR_CHECK(ret);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Read 'valid_samples' data and calculate the average
    for (int i = 0; i < valid_samples; i++) {
        int temp_value;
        esp_err_t ret = adc_oneshot_read(adc_handle, GRAY_CHANNEL, &temp_value);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ADC read timeout during valid phase");
            continue; // Skip this reading
        }
        ESP_ERROR_CHECK(ret);
        raw_value += temp_value;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    raw_value /= valid_samples; // Calculate the average value

    ESP_LOGI(TAG, "Gray Sensor ADC Value (filtered): %d", raw_value);
    return (uint32_t)raw_value;
}