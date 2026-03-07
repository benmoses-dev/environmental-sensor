idf_component_register(
    SRCS "src/main.cpp" "src/bme280.cpp" "src/wifi.cpp" "src/mqtt.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_wifi nvs_flash mqtt mbedtls
)
