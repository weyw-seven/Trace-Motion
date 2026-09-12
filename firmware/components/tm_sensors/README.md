# `tm_sensors`

面向多个任务复用的基础传感器组件。

| 接口 | 职责 |
| --- | --- |
| `infrared_sensor.h` | 四路红外 GPIO 读取、黑线有效电平归一化和模式输出。 |
| `ultrasonic_sensor.h` | 超声触发、RMT ECHO 捕获、中值滤波和障碍物判定。 |

实现仅在 `tm_sensors.c` 中编译一次。红外引脚顺序、有效电平、超声阈值和采样周期是实车相关参数，测量方法见 [硬件配置与标定指南](../../../docs/HARDWARE_CALIBRATION.md)。
