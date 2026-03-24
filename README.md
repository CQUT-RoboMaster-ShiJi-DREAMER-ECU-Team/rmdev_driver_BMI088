# rmdev_driver_BMI088

BMI088 驱动模块（`rmdev` 子模块）。

## 依赖

- `emdevif_core`
- `emdevif_peripheral`
- `emdevif_timeline`
- `rmdev_device_model`

## 配置

- `RMDEV_DRIVER_BMI088_USE_SPI`（默认 `ON`）
  - `ON`：使用 SPI
  - `OFF`：使用 I2C

## 使用流程（建议）

1. 在上层启用驱动：`RMDEV_ENABLED_DRIVER_LIST="BMI088"`
2. 根据硬件选择 `SPI/I2C` 配置
3. 在 `emdevif_user_declares` 中完成外设句柄映射
4. 将读取结果写入 `rmdev_device_model` 的 IMU 模型
