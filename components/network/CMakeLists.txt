idf_component_register(
    SRCS "src/wifi.cpp" "src/mqtt.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi nvs_flash mqtt mbedtls
)
