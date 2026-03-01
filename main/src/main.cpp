#include "bme280.hpp"
#include "esp_log.h"

static const char *TAG = "MAIN";

BME280 bme;

extern "C" void app_main() {
    if (!bme.init()) {
        ESP_LOGE(TAG, "BME280 failed to initialise!");
        return;
    }
    while (true) {
        ESP_LOGI(TAG, "Temp: %.2f °C", bme.readTemperature());
        ESP_LOGI(TAG, "Humidity: %.2f %%", bme.readHumidity());
        ESP_LOGI(TAG, "Pressure: %.2f hPa", bme.readPressure() / 100.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
