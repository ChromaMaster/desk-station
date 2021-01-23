idf_component_register(
        SRCS "ds_temp_sensor.c"
        INCLUDE_DIRS "include"
        REQUIRES "driver ds_message ds_log"
)

# Link BME280 Library
add_subdirectory(BME280_driver)
target_link_libraries(${COMPONENT_LIB} PRIVATE BME280_driver)