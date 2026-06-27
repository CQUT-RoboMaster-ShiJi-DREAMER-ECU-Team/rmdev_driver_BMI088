# rmdev_driver_BMI088

BMI088 IMU 驱动模块（`rmdev` 子模块）。

## 依赖

- `emdevif_core`
- `emdevif_peripheral`
- `emdevif_timeline`
- `rmdev_device_model`

## 配置

- `RMDEV_DRIVER_BMI088_USE_SPI`（默认 `ON`）
  - `ON`：使用 SPI 总线访问 BMI088
  - `OFF`：使用 I2C 总线访问 BMI088

## 启用方式

在上层 `rmdev` 中通过驱动列表启用：

```cmake
set(RMDEV_ENABLED_DRIVER_LIST "BMI088" CACHE INTERNAL "" FORCE)
# 也可与其他驱动组合，例如："BMI088;DJIMotor"
```

## 使用流程

1. 在上层 `rmdev` 中通过 `RMDEV_ENABLED_DRIVER_LIST` 启用本驱动。
2. 根据硬件选择 `SPI` / `I2C` 配置（`RMDEV_DRIVER_BMI088_USE_SPI`）。
3. 通过**链接期注入**完成外设句柄映射：在任意 `.cpp` 文件中实现
   `emdevif::user_impl::peripheral_handle_map::findHandle`，将本驱动用到的外设名称（如 SPI 句柄）映射到对应的 `XxxModel::Instance`。具体注册方式见
   [`emdevif/peripheral/README.md`](../../../emdevif_collection/emdevif/peripheral/README.md)。
4. 将读取结果写入 `rmdev_device_model` 的 IMU 模型，供算法层消费。
