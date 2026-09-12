# ESP32-S3 固件

本目录是 Trace Motion（追迹）的 ESP-IDF 多应用工作区。`apps/` 下每个目录都是独立工程根目录；请进入目标应用目录运行 `idf.py`，不要在 `firmware/` 目录直接构建。

| 应用 | 用途 | 构建状态 |
| --- | --- | --- |
| `apps/trajectory-drawing/` | TRJ2 上传、校验、轨迹执行和安全档 | 可作为参考工程构建 |
| `apps/line-infrared/` | 四路红外循迹、超声避障和 OLED 状态显示；包含完整任务实现 | 可构建，需按实际车辆验证 |
| `apps/line-ball-camera/` | 相机循迹、找球与推球 | 源码已整理，尚缺可复现工程配置 |

`tests/` 存放独立构建的底盘标定与硬件验收程序，不是日常任务应用。参见 [测试说明](tests/README.md)。

## 组件布局

公共实现位于 `components/`，由 ESP-IDF 的 `EXTRA_COMPONENT_DIRS` 发现：

| 组件 | 职责 |
| --- | --- |
| `tm_chassis` | 电机、MPU6050、三轮运动学、里程计与底盘运动 |
| `tm_sensors` | 红外和超声驱动 |
| `tm_trajectory` | TRJ2 解码、执行、跟踪与笔控制接口 |
| `tm_n3` | N3 服务、传输、轨迹运行桥接和硬件初始化 |

红外循线、避障和 OLED 属于 `apps/line-infrared/main/` 的任务专用实现；底盘与传感器等跨任务能力才放在 `components/`。应用内的 `line_infrared_app.c` 是这些单头文件模块唯一启用实现宏的位置。

## 构建

安装 ESP-IDF 5.4.4 并加载环境后，例如构建轨迹绘图应用：

```powershell
cd firmware\apps\trajectory-drawing
idf.py set-target esp32s3
idf.py build
```

构建参数、应用职责和安全约束见仓库根目录 [README](../README.md) 与各应用目录中的 README。
