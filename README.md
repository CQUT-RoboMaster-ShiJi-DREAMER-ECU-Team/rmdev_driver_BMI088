# rmdev_driver_BMI088

属于 [rmdev](https://github.com/CQUT-RoboMaster-ShiJi-DREAMER-ECU-Team/rmdev.git) 的一个子模块，是 BMI088 的驱动。

## 依赖

* `emdevif`（包含 `emdevif_peripheral` 和 `emdevif_timeline`）
* `rmdev_device_model`

## 配置

CMake 缓存变量：
* `RMDEV_DRIVER_BMI088_USE_SPI`: 设置为 `ON`，使用 SPI 通信；设置为 `OFF`，使用 I2C。
