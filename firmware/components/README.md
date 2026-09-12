# 公共组件规划

这里将只放已经稳定、可跨应用复用、拥有明确 API 和测试的 ESP-IDF 组件。计划中的组件包括：

- `tm_motor_control`：电机、编码器、急停与板级配置。
- `tm_chassis`：三轮运动学、里程计与通用底盘运动接口。
- `tm_sensors`：MPU6050、超声波与 OLED 的设备驱动。
- `tm_trajectory_protocol`：TRJ2 和 N3 通信协议。

不要直接把 task2 的同名头文件移动到这里。目前它们与绘图应用存在不同控制参数、接口依赖和实现行为。先将单头文件实现拆分为 `include/`、`.c`、`CMakeLists.txt` 与 `idf_component.yml`，再以版本标签供应用依赖。
