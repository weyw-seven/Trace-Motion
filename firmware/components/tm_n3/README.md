# `tm_n3`

PC 与 ESP32 之间的 N3 服务组件，用于轨迹上传、命令处理、状态回传和受控执行。

| 接口 | 职责 |
| --- | --- |
| `n3_service.h` | 服务生命周期、JSON 命令和状态消息。 |
| `n3_transport.h` | USB Serial/JTAG、UART 与可选 Wi-Fi TCP 传输。 |
| `n3_runner_bridge.h` | TRJ2 文件预检、执行桥接、Stop 与 E-Stop 状态。 |
| `n3_motion.h`、`n3_hardware.h` | 运动和硬件安全边界。 |

`trajectory-drawing` 默认使用安全档：可通信、上传和校验，但不驱动实物。启用电机、笔机构或 Wi-Fi 前，必须在构建命令中显式设置 `N3_ENABLE_*` 参数，并完成底盘和急停验证。
