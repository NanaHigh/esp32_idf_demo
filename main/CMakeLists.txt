idf_component_register(SRCS "app_main.c" "./src/device.c" "./src/bmp280.c" "./src/bh1750.c"
 "./src/gray.c" "./src/icm42688.c"  "./src/mq2.c" "./src/radar.c"
                    INCLUDE_DIRS "." "./src" "./lib")
