#include "device.h"
#include "radar.h"

static const char *TAG = "radar";

bool read_radar_status(void)
{
    int level = gpio_get_level(RADAR_PIN);
    if (level == 1) {
        ESP_LOGI(TAG, "Radar status: HIGH");
        return true;
    } else {
        ESP_LOGI(TAG, "Radar status: LOW");
        return false;
    }
}