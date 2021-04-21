idf_component_register(SRCS "esp32_nat_router.c"
                            "http_server.c" 
                            "led/led_strip_rmt_ws2812.c"
                            "led/led_strip_main.c"
                            "ledc/ledc_main.c"
                            "gpio5_relay/gpio5_relay.c"
                            "dht11/dht11_main.c"
                            "mqtt/mqtt_main.c"
                            "urat/uart_echo.c"
                            "AGS02MA/ags02ma.c"
                            
                        INCLUDE_DIRS "include"
                        INCLUDE_DIRS ".")
