/**
 * @file Bmi088.cppm
 * @brief BMI088 驱动
 */

// ReSharper disable CppExpressionWithoutSideEffects

module;

#include <cstdint>
#include <cmath>

#include <array>
#include <limits>

#include "rmdev/driver/bmi088/BMI088reg.h"
#include "emdevif/core/fatal_handler.h"

#define EMDEVIF_MODULE_INTERFACE_UNIT

export module rmdev.driver.imu.bmi088;

import emdevif.core.type_traits;
import emdevif.core.concepts;
import emdevif.core.error_handler;
import emdevif.core.integer_suffix;
import rmdev.device_model.sensor.imu;

#if (RMDEV_DRIVER_BMI088_USE_SPI)
import emdevif.peripheral.spi;
import emdevif.peripheral.gpio;
import emdevif.timeline;
#else
import emdevif.peripheralModels.i2c;
#endif

#ifdef __clang__
    #pragma clang diagnostic ignored "-Winclude-angled-in-module-purview"
#endif

#include "rmdev/driver/bmi088.hpp"
