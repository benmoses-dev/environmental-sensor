idf_component_register(
    SRCS "src/environment.cpp" "src/bme280.cpp" "src/bme680.cpp" "src/bme68x.c" "src/sds011.cpp" "src/scd41.cpp" "src/sgp40.cpp" "src/sgp41.cpp" "src/sht45.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_uart esp_timer
)
