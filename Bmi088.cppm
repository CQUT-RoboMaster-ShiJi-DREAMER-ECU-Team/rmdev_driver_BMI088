/**
 * @file Bmi088.cppm
 * @brief BMI088 驱动
 */

// ReSharper disable CppExpressionWithoutSideEffects

module;

#include "rmdev/driver/bmi088.hpp"

export module rmdev.driver.bmi088;

export namespace rmdev {
    using ::rmdev::init_fail_max_count;
    using ::rmdev::Bmi088;
}
