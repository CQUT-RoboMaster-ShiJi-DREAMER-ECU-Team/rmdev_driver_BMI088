cmake_minimum_required(VERSION 3.28...4.0.2)

project(rmdev_driver_BMI088 C CXX)

set(RMDEV_DRIVER_BMI088_USE_SPI ON CACHE BOOL "Set to ON: use SPI\; OFF: use I2C.")

if (EMDEVIF_USE_CPP_MODULES)
    add_library(${PROJECT_NAME} STATIC)

    target_include_directories(${PROJECT_NAME} PUBLIC inc)
    target_sources(${PROJECT_NAME}
        PUBLIC FILE_SET ${PROJECT_NAME}_modules TYPE CXX_MODULES FILES Bmi088.cppm
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
else ()
    add_library(${PROJECT_NAME} INTERFACE)
    target_include_directories(${PROJECT_NAME} INTERFACE inc)
    target_link_libraries(${PROJECT_NAME} INTERFACE emdevif_core emdevif_compile_options
        emdevif_peripheral emdevif_timeline rmdev_device_model
    )

    if (RMDEV_DRIVER_BMI088_USE_SPI)
        target_compile_definitions(${PROJECT_NAME} INTERFACE RMDEV_DRIVER_BMI088_USE_SPI=1)
    else ()
        target_compile_definitions(${PROJECT_NAME} INTERFACE RMDEV_DRIVER_BMI088_USE_SPI=0)
    endif ()
endif ()
