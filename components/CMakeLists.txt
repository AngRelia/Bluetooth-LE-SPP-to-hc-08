set(src_dirs
            LED
            delay
            oled
            GPTIM
            KEY
            Serial
            Bluetooth)

set(include_dirs
            LED
            delay
            oled
            GPTIM
            KEY
            Serial
            Bluetooth)

set(requires
            driver
            esp_timer
            nvs_flash
            bt)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})



