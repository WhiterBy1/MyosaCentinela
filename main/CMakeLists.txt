# --- Firmware de campo combinado: sensores I2C + LoRa (Wio-E5) ---
idf_component_register(SRCS "fall_model_wrap.cpp" "myosa_field_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash lwip
                                     esp_timer esp_driver_uart esp_driver_gpio esp_adc)

# --- Firmware de sensores solo (sin LoRa) ---
# idf_component_register(SRCS "i2c_basic_example_main.c"
#                        INCLUDE_DIRS ".")

# --- Prueba aislada del APDS9960 (ya no en uso) ---
# idf_component_register(SRCS "gesture_test_main.c"
#                        INCLUDE_DIRS ".")

# --- Prueba aislada del Wio-E5 por UART (AT + TX P2P) ---
# idf_component_register(SRCS "lora_wioe5_test_main.c"
#                        INCLUDE_DIRS ".")
