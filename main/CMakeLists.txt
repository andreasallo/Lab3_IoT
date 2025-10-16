idf_component_register(SRCS "led_strip_example_main.c" "led_strip_encoder.c"
                       PRIV_REQUIRES esp_driver_rmt driver
                       INCLUDE_DIRS ".")