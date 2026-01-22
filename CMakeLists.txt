cmake_minimum_required(VERSION 3.28...4.0.2)

project(rmdev_driver_BMI088 C CXX)

set(RMDEV_DRIVER_BMI088_USE_SPI ON CACHE BOOL "Set to ON: use SPI\; OFF: use I2C.")

add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME}
    PUBLIC FILE_SET ${PROJECT_NAME}_modules TYPE CXX_MODULES FILES
        Bmi088.cppm

    PRIVATE FILE_SET ${PROJECT_NAME}_headers TYPE HEADERS BASE_DIRS impl/ FILES
        impl/BMI088reg.h
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC emdevif_core
    PRIVATE emdevif_compile_options emdevif_peripheral emdevif_timeline
        rmdev_device_model
)

if (RMDEV_DRIVER_BMI088_USE_SPI)
    target_compile_definitions(${PROJECT_NAME} PRIVATE RMDEV_DRIVER_BMI088_USE_SPI=1)
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE RMDEV_DRIVER_BMI088_USE_SPI=0)
endif ()
