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

#include "BMI088reg.h"

#include "emdevif/fatal_handler.h"
#include "emdevif/integer_suffix.hpp"

#ifndef RMDEV_DRIVER_BMI088_USE_SPI
#error \
    "Please add a defination `RMDEV_DRIVER_BMI088_USE_SPI\' to decide how to connect with BMI088. Set it to true: use SPI; false: use I2C."
#endif

#ifndef RMDEV_DRIVER_BMI088_INIT_FAIL_MAX_COUNT
#define RMDEV_DRIVER_BMI088_INIT_FAIL_MAX_COUNT (std::numeric_limits<std::size_t>::max())
#endif

export module rmdev.driver.imu.bmi088;

import emdevif.typeTraits;
import emdevif.concepts;
import emdevif.errorHandler;
export import rmdev.deviceModel.sensor.imu;

#if (RMDEV_DRIVER_BMI088_USE_SPI)
import emdevif.peripheral.spi;
import emdevif.peripheral.gpio;
import emdevif.timeline;
#else
import emdevif.peripheralModels.i2c;
#endif

namespace rmdev::inline drivers {

// 需手动修改
#define GxOFFSET 0.001894738333f
#define GyOFFSET (-0.00337262948f)
#define GzOFFSET (-0.00135726751f)
#define gNORM    9.7229061133f

export constexpr std::size_t init_fail_max_count = RMDEV_DRIVER_BMI088_INIT_FAIL_MAX_COUNT;

export template<emdevif::ArithmeticType T = float>
class Bmi088
{
public:
    using DataType = T;

public:
    /**
     * 获取 IMU 数据（只有角速度、加速度、温度的数据有效）
     * @return IMU 数据的引用
     */
    constexpr rmdev::Imu<DataType>& getImuData() noexcept
    {
        return static_cast<rmdev::Imu<DataType>&>(imu_data_);
    }
    /**
     * @overload
     * @return IMU 数据常量引用
     */
    constexpr const rmdev::Imu<DataType>& getImuData() const noexcept
    {
        return static_cast<rmdev::Imu<DataType>&>(imu_data_);
    }

    /**
     * 设备初始化
     * @param calibrate 是否进行校准
     */
    void deviceInit(const bool calibrate = true) noexcept
    {
        using namespace emdevif::literals;

        auto fail_count = 0_zu;
        while (deviceInitImpl(calibrate) != emdevif::ErrorCode::Success) {
            ++fail_count;

            if (fail_count >= init_fail_max_count) {
                EMDEVIF_FATAL_HANDLER("Failed to init BMI088 device.");
            }
        }
    }

    /**
     * 从传感器处读取 IMU 数据
     */
    void readImuData() noexcept
    {
        readImuDataImpl();
    }

#if (RMDEV_DRIVER_BMI088_USE_SPI)
private:
    emdevif::Spi spi_;
    emdevif::Gpio cs_accel_;
    emdevif::Gpio cs_gyro_;

private:
    struct Bmi088Data : public rmdev::Imu<DataType> {
        DataType temp_when_cali;
        DataType accel_scale;

        DataType gyro_offset[3];

        DataType g_norm;
    } imu_data_;

    constexpr Bmi088Data& getFullData() noexcept
    {
        return imu_data_;
    }
    constexpr const Bmi088Data& getFullData() const noexcept
    {
        return imu_data_;
    }

public:
    constexpr Bmi088(const emdevif::Spi& spi, const emdevif::Gpio& cs_accel, const emdevif::Gpio& cs_gyro) noexcept
        : spi_(spi), cs_accel_(cs_accel), cs_gyro_(cs_gyro)
    {
    }

private:
    void accelCsL() const noexcept
    {
        cs_accel_.write(emdevif::Gpio::Reset);
    }

    void accelCsH() const noexcept
    {
        cs_accel_.write(emdevif::Gpio::Set);
    }

    void gyroCsL() const noexcept
    {
        cs_gyro_.write(emdevif::Gpio::Reset);
    }

    void gyroCsH() const noexcept
    {
        cs_gyro_.write(emdevif::Gpio::Set);
    }

    uint8_t readWriteByte(uint8_t wrtie_value) const noexcept  // NOLINT(*-use-nodiscard)
    {
        uint8_t read_value;
        (void)spi_.transmitReceive(false, {&wrtie_value, 1}, {&read_value, 1}, 1000);
        return read_value;
    }

    template<std::size_t length>
        requires(length <= 8)
    void readMuliReg(const uint8_t reg, uint8_t* data) const noexcept
    {
        (void)readWriteByte(reg | 0x80U);

        constexpr std::array<uint8_t, 8> tx_buffer{0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

        spi_.transmitReceive(false, {tx_buffer.data(), length}, {data, length}, 1000);
    }

    static constexpr auto BMI088_TEMP_FACTOR = 0.125f;
    static constexpr auto BMI088_TEMP_OFFSET = 23.0f;

    static constexpr auto BMI088_WRITE_ACCEL_REG_NUM = 6;
    static constexpr auto BMI088_WRITE_GYRO_REG_NUM = 6;

    static constexpr auto BMI088_GYRO_DATA_READY_BIT = 0;
    static constexpr auto BMI088_ACCEL_DATA_READY_BIT = 1;
    static constexpr auto BMI088_ACCEL_TEMP_DATA_READY_BIT = 2;

    static constexpr auto BMI088_LONG_DELAY_TIME = 120;
    static constexpr auto BMI088_COM_WAIT_SENSOR_TIME = 150;

    static constexpr auto BMI088_ACCEL_IIC_ADDRESSE = (0x18 << 1);
    static constexpr auto BMI088_GYRO_IIC_ADDRESSE = (0x68 << 1);

    static constexpr auto BMI088_ACCEL_3G_SEN = 0.0008974358974f;
    static constexpr auto BMI088_ACCEL_6G_SEN = 0.00179443359375f;
    static constexpr auto BMI088_ACCEL_12G_SEN = 0.0035888671875f;
    static constexpr auto BMI088_ACCEL_24G_SEN = 0.007177734375f;

    static constexpr auto BMI088_GYRO_2000_SEN = 0.00106526443603169529841533860381f;
    static constexpr auto BMI088_GYRO_1000_SEN = 0.00053263221801584764920766930190693f;
    static constexpr auto BMI088_GYRO_500_SEN = 0.00026631610900792382460383465095346f;
    static constexpr auto BMI088_GYRO_250_SEN = 0.00013315805450396191230191732547673f;
    static constexpr auto BMI088_GYRO_125_SEN = 0.000066579027251980956150958662738366f;

    enum Status : uint8_t {
        BMI088_NO_ERROR = 0x00,
        BMI088_ACC_PWR_CTRL_ERROR = 0x01,
        BMI088_ACC_PWR_CONF_ERROR = 0x02,
        BMI088_ACC_CONF_ERROR = 0x03,
        BMI088_ACC_SELF_TEST_ERROR = 0x04,
        BMI088_ACC_RANGE_ERROR = 0x05,
        BMI088_INT1_IO_CTRL_ERROR = 0x06,
        BMI088_INT_MAP_DATA_ERROR = 0x07,
        BMI088_GYRO_RANGE_ERROR = 0x08,
        BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
        BMI088_GYRO_LPM1_ERROR = 0x0A,
        BMI088_GYRO_CTRL_ERROR = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

        BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
        BMI088_SELF_TEST_GYRO_ERROR = 0x40,
        BMI088_NO_SENSOR = 0xFF
    };

    static constexpr uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL,
         BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
         BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};

    static constexpr uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF,
         BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
         BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};

    static constexpr float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
    static constexpr float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

    uint8_t error = BMI088_NO_ERROR;

    void BMI088_write_single_reg(const uint8_t reg, const uint8_t data) const noexcept
    {
        readWriteByte(reg);
        readWriteByte(data);
    }

    void BMI088_read_single_reg(const uint8_t reg, uint8_t* return_data) const noexcept
    {
        readWriteByte(reg | 0x80);
        *return_data = readWriteByte(0x55);
    }

#define BMI088_accel_write_single_reg(reg, data) \
    do {                                         \
        accelCsL();                              \
        BMI088_write_single_reg((reg), (data));  \
        accelCsH();                              \
    } while (0)

#define BMI088_accel_read_single_reg(reg, data) \
    do {                                        \
        accelCsL();                             \
        readWriteByte((reg) | 0x80);            \
        readWriteByte(0x55);                    \
        (data) = readWriteByte(0x55);           \
        accelCsH();                             \
    } while (0)

    // #define BMI088_accel_write_muli_reg( reg,  data, len) { accelCsL(); BMI088_write_muli_reg(reg, data, len);
    // accelCsH(); }

#define BMI088_accel_read_muli_reg(reg, data, len) \
    do {                                           \
        accelCsL();                                \
        readWriteByte((reg) | 0x80);               \
        readMuliReg<len>(reg, data);               \
        accelCsH();                                \
    } while (0)

#define BMI088_gyro_write_single_reg(reg, data) \
    do {                                        \
        gyroCsL();                              \
        BMI088_write_single_reg((reg), (data)); \
        gyroCsH();                              \
    } while (0)

#define BMI088_gyro_read_single_reg(reg, data)  \
    do {                                        \
        gyroCsL();                              \
        BMI088_read_single_reg((reg), &(data)); \
        gyroCsH();                              \
    } while (0)

    // #define BMI088_gyro_write_muli_reg( reg,  data, len) { gyroCsL(); BMI088_write_muli_reg( ( reg ), ( data
    // ), ( len ) ); gyroCsH(); }

#define BMI088_gyro_read_muli_reg(reg, data, len) \
    do {                                          \
        gyroCsL();                                \
        readMuliReg<len>((reg), (data));          \
        gyroCsH();                                \
    } while (0)

    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    Status bmi088_accel_init() noexcept
    {
        // check commiunication
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
        emdevif::Timeline::pauseDelayMs(1);
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
        emdevif::Timeline::pauseDelayMs(1);

        // accel software reset
        BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
        emdevif::Timeline::pauseDelayMs(BMI088_LONG_DELAY_TIME);

        // check commiunication is normal after reset
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
        emdevif::Timeline::pauseDelayMs(1);
        BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
        emdevif::Timeline::pauseDelayMs(1);

        // check the "who am I"
        if (res != BMI088_ACC_CHIP_ID_VALUE) return BMI088_NO_SENSOR;
        // do
        // {
        //     BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
        // } while (res != BMI088_ACC_CHIP_ID_VALUE);

        // set accel sonsor config and check
        for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
            BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0],
                                          write_BMI088_accel_reg_data_error[write_reg_num][1]);
            emdevif::Timeline::pauseDelayMs(1);

            BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
            emdevif::Timeline::pauseDelayMs(1);

            if (res != write_BMI088_accel_reg_data_error[write_reg_num][1]) {
                // write_reg_num--;
                // return write_BMI088_accel_reg_data_error[write_reg_num][2];
                error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
            }
        }
        return BMI088_NO_ERROR;
    }

    Status bmi088_gyro_init() noexcept
    {
        // check commiunication
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
        emdevif::Timeline::pauseDelayMs(1);
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
        emdevif::Timeline::pauseDelayMs(1);

        // reset the gyro sensor
        BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
        emdevif::Timeline::pauseDelayMs(BMI088_LONG_DELAY_TIME);
        // check commiunication is normal after reset
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
        emdevif::Timeline::pauseDelayMs(1);
        BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
        emdevif::Timeline::pauseDelayMs(1);

        // check the "who am I"
        if (res != BMI088_GYRO_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

        // set gyro sonsor config and check
        for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
            BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0],
                                         write_BMI088_gyro_reg_data_error[write_reg_num][1]);
            emdevif::Timeline::pauseDelayMs(1);

            BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
            emdevif::Timeline::pauseDelayMs(1);

            if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1]) {
                write_reg_num--;
                // return write_BMI088_gyro_reg_data_error[write_reg_num][2];
                error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
            }
        }

        return BMI088_NO_ERROR;
    }

    float gyroDiff[3];
    float gNormDiff;
    int16_t caliCount = 0;

    void Calibrate_MPU_Offset(Bmi088Data* bmi088) noexcept
    {
        DataType startTime;
        uint16_t CaliTimes = 6000;  // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
        uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
        int16_t bmi088_raw_temp;
        DataType gyroMax[3], gyroMin[3];
        DataType gNormTemp, gNormMax, gNormMin;

        constexpr auto getTimelineSeconds = []() noexcept -> DataType {
            return static_cast<DataType>(emdevif::Timeline::getMicroseconds()) / static_cast<DataType>(1'000'000.0);
        };

        startTime = getTimelineSeconds();
        do {
            if (getTimelineSeconds() - startTime > 15) {
                // 校准超时
                bmi088->gyro_offset[0] = GxOFFSET;
                bmi088->gyro_offset[1] = GyOFFSET;
                bmi088->gyro_offset[2] = GzOFFSET;
                bmi088->g_norm = gNORM;

                BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
                bmi088_raw_temp = static_cast<int16_t>((buf[0] << 3) | (buf[1] >> 5));
                if (bmi088_raw_temp > 1023) bmi088_raw_temp -= 2048;
                bmi088->temp_when_cali = static_cast<float>(bmi088_raw_temp) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

                break;
            }

            emdevif::Timeline::pauseDelayMs(5);
            bmi088->g_norm = 0;
            bmi088->gyro_offset[0] = 0;
            bmi088->gyro_offset[1] = 0;
            bmi088->gyro_offset[2] = 0;

            for (uint16_t i = 0; i < CaliTimes; i++) {
                BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
                bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
                bmi088->accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
                gNormTemp = sqrtf(bmi088->accel[0] * bmi088->accel[0] + bmi088->accel[1] * bmi088->accel[1] +
                                  bmi088->accel[2] * bmi088->accel[2]);
                bmi088->g_norm += gNormTemp;

                BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
                if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
                    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                    bmi088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                    bmi088->gyro_offset[0] += bmi088->gyro[0];
                    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                    bmi088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                    bmi088->gyro_offset[1] += bmi088->gyro[1];
                    bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                    bmi088->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                    bmi088->gyro_offset[2] += bmi088->gyro[2];
                }

                // 记录数据极差
                if (i == 0) {
                    gNormMax = gNormTemp;
                    gNormMin = gNormTemp;
                    for (uint8_t j = 0; j < 3; j++) {
                        gyroMax[j] = bmi088->gyro[j];
                        gyroMin[j] = bmi088->gyro[j];
                    }
                }
                else {
                    if (gNormTemp > gNormMax) gNormMax = gNormTemp;
                    if (gNormTemp < gNormMin) gNormMin = gNormTemp;
                    for (uint8_t j = 0; j < 3; j++) {
                        if (bmi088->gyro[j] > gyroMax[j]) gyroMax[j] = bmi088->gyro[j];
                        if (bmi088->gyro[j] < gyroMin[j]) gyroMin[j] = bmi088->gyro[j];
                    }
                }

                // 数据差异过大认为收到外界干扰，需重新校准
                gNormDiff = gNormMax - gNormMin;
                for (uint8_t j = 0; j < 3; j++) gyroDiff[j] = gyroMax[j] - gyroMin[j];
                if (gNormDiff > 0.5f || gyroDiff[0] > 0.15f || gyroDiff[1] > 0.15f || gyroDiff[2] > 0.15f) break;
                emdevif::Timeline::pauseDelayUs(500);
            }

            // 取平均值得到标定结果
            bmi088->g_norm /= (DataType)CaliTimes;
            for (uint8_t i = 0; i < 3; i++) bmi088->gyro_offset[i] /= (DataType)CaliTimes;

            // 记录标定时IMU温度
            BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
            bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
            if (bmi088_raw_temp > 1023) bmi088_raw_temp -= 2048;
            bmi088->temp_when_cali = static_cast<float>(bmi088_raw_temp) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

            caliCount++;
        } while (gNormDiff > 0.5f || std::abs(bmi088->g_norm - 9.8f) > 0.5f || gyroDiff[0] > 0.15f ||
                 gyroDiff[1] > 0.15f || gyroDiff[2] > 0.15f || std::abs(bmi088->gyro_offset[0]) > 0.01f ||
                 std::abs(bmi088->gyro_offset[1]) > 0.01f || std::abs(bmi088->gyro_offset[2]) > 0.01f);

        // 根据标定结果校准加速度计标度因数
        bmi088->accel_scale = 9.81f / bmi088->g_norm;
    }

    Status BMI088_init(const bool calibrate) noexcept
    {
        error = BMI088_NO_ERROR;

        error |= bmi088_accel_init();
        error |= bmi088_gyro_init();

        if (calibrate) {
            Calibrate_MPU_Offset(&getFullData());
        }
        else {
            getFullData().gyro_offset[0] = GxOFFSET;
            getFullData().gyro_offset[1] = GyOFFSET;
            getFullData().gyro_offset[2] = GzOFFSET;
            getFullData().g_norm = gNORM;
            getFullData().accel_scale = 9.81f / getFullData().g_norm;
            getFullData().temp_when_cali = 40;
        }

        return static_cast<Status>(error);
    }

    emdevif::ErrorCode deviceInitImpl(const bool calibrate) noexcept
    {
        if (BMI088_init(calibrate) == BMI088_NO_ERROR) {
            return emdevif::ErrorCode::Success;
        }

        return emdevif::ErrorCode::OperationFail;
    }

    uint8_t caliOffset = 1;

    void BMI088_Read(Bmi088Data* bmi088)
    {
        static uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
        static int16_t bmi088_raw_temp;

        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
        bmi088->accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->accel_scale;
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        bmi088->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->accel_scale;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        bmi088->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->accel_scale;

        BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
        if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
            if (caliOffset) {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->gyro_offset[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->gyro_offset[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->gyro_offset[2];
            }
            else {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->gyro[2] = static_cast<float>(bmi088_raw_temp) * BMI088_GYRO_SEN;
            }
        }
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

        bmi088_raw_temp = static_cast<int16_t>((buf[0] << 3) | (buf[1] >> 5));

        if (bmi088_raw_temp > 1023) {
            bmi088_raw_temp -= 2048;
        }

        bmi088->temperature = static_cast<float>(bmi088_raw_temp) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    void readImuDataImpl() noexcept
    {
        BMI088_Read(&getFullData());
    }

#elif
#endif
};

}  // namespace rmdev::inline drivers
