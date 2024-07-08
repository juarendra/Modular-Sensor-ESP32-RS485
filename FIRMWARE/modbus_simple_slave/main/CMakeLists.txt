set(PROJECT_NAME "modbus_slave")

idf_component_register(SRCS "slave.cpp" "ModbusSlave.cpp"
                    INCLUDE_DIRS ".")

spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)