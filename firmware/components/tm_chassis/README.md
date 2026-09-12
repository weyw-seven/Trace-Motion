# `tm_chassis`

三轮全向底盘的公共控制组件，供红外任务、轨迹绘图和硬件测试复用。

| 接口 | 职责 |
| --- | --- |
| `motor_control.h` | PWM、PCNT 编码器、方向相关前馈和速度 PID。 |
| `mpu6050.h` | IMU 初始化、静止校准和姿态快照。 |
| `chassis_kinematics.h` | 三轮正、逆运动学。 |
| `chassis_odometry.h` | 编码器与航向融合的世界坐标里程计。 |
| `chassis_motion.h` | 移动、横移和旋转等非阻塞运动基元。 |

实现仅在 `tm_chassis.c` 中编译一次。电机引脚、轮径、编码器极性和控制参数必须按实车配置，修改后运行 `firmware/tests/hardware/` 中对应测试。
