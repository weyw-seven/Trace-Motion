# 硬件配置与标定指南

这份文档列出 Trace Motion 在更换底盘、轮子、传感器或接线后需要复核的变量，以及获得这些数值的测量方法。所有数值应记录在对应公共组件的头文件中；不要把不同车辆的参数散落在任务应用或测试程序中。

## 修改位置总览

| 对象 | 文件 | 需要关注的变量 |
| --- | --- | --- |
| 电机、编码器、车体尺寸和轮速控制 | `firmware/components/tm_chassis/include/motor_control.h` | `MOTOR_*` |
| 三轮里程计 | `firmware/components/tm_chassis/include/chassis_odometry.h` | `CHASSIS_ODOMETRY_*` |
| 底盘闭环运动 | `firmware/components/tm_chassis/include/chassis_motion.h` | `CHASSIS_MOTION_*` |
| MPU6050 | `firmware/components/tm_chassis/include/mpu6050.h` | `MPU6050_*` |
| 红外循迹传感器 | `firmware/components/tm_sensors/include/infrared_sensor.h` | `IR_SENSOR_*` |
| 超声传感器 | `firmware/components/tm_sensors/include/ultrasonic_sensor.h` | `ULTRASONIC_*` |
| 红外循迹和避障行为 | `firmware/apps/line-infrared/main/line_tracker.h`、`obstacle_avoid.h` | `LINE_*`、`AVOID_*` |
| OLED | `firmware/apps/line-infrared/main/oled_display.h` | `OLED_*` |
| 抬笔机构 | `firmware/components/tm_trajectory/include/pen_control.h` 与 `firmware/components/tm_n3/n3_runner_bridge.c` | `PEN_CONTROL_DEFAULT_*`、笔 GPIO 与标定状态 |

## 推荐顺序

按下列顺序完成，而不是直接调高层循迹或绘图参数：

1. 核对供电、接线与 GPIO；确认没有引脚冲突。
2. 标定编码器计数、轮径、轮子方向和 IMU 方向。
3. 用启动阈值和前馈测试标定单轮输出。
4. 用小距离运动和手推里程计测试验证底盘。
5. 标定超声、红外、OLED 与笔机构。
6. 最后调整循迹、避障和轨迹运动控制参数。

每次仅修改一类变量并提交一次可追溯的记录。测量时使用同一块电池、轮胎、载荷和地面；这些条件变化会影响前馈和 PID。

## 1. 电机、编码器和几何

配置文件：`tm_chassis/include/motor_control.h`

| 变量组 | 如何获得 | 验证 |
| --- | --- | --- |
| `MOTOR_STBY_PIN`、`MOTOR_[A/B/D]_{IN1,IN2,PWM}` | 对照实际驱动板的 STBY、方向和 PWM 接线逐根填写。 | 上电前用万用表检查；再以低 PWM 单轮测试方向。 |
| `MOTOR_[A/B/D]_ENC_{A,B}` | 对照每个编码器 A/B 相接线填写。 | 运行启动阈值测试，确认每轮都有稳定计数。 |
| `MOTOR_[A/B/D]_ENCODER_CPR` | 让单个轮子慢速转 **至少 10 圈**，记录编码器累计计数绝对值，取 `counts / 圈数`；三轮分别测量。这里是减速后、PCNT 实际看到的每轮一圈计数。 | 用相同轮子再转 10 圈，估算值与实测计数误差应小。 |
| `MOTOR_WHEEL_DIAMETER_MM` | 用卡尺测轮胎实际滚动直径，或让车直行 1–2 m 后用卷尺测真实距离。若估算距离为 `L_est`、真实距离为 `L_true`，使用 `D_new = D_old × L_true / L_est`。 | 使用 `odometry-push` 沿直线手推，比较里程计和卷尺。 |
| `MOTOR_CHASSIS_RADIUS_MM` | 测量车体运动中心到每个轮子接地点的距离，取三者平均值。 | 低速原地旋转，观察位姿与真实旋转角；半径偏差会导致角速度和横移耦合异常。 |
| `MOTOR_[A/B/D]_POLARITY` | 悬空时给每轮低速正命令。若轮子方向与软件约定相反，将对应值设为 `-1.0f`。 | 使用 `small-motion` 检查前进、横移、斜向和回退方向。 |
| `MOTOR_[A/B/D]_ENCODER_POLARITY` | 在电机正向运动时观察编码器计数与实际速度符号；若相反设为 `-1.0f`。 | `startup-threshold` 与 `odometry-push` 的正反向计数应一致。 |
| `MOTOR_[A/B/D]_{KX,KY,KW}` | 标准 120° 三轮布局可保留当前矩阵。改变轮子编号、安装角度或车体坐标定义时，按每个轮子滚动方向重新推导逆运动学。 | 依次验证 `+X` 前进、`+Y` 左移和正角速度逆时针旋转。 |

`MOTOR_A/B/D_PWM_LIMIT` 是电源、驱动器与电机的保护上限。首次使用较低值，确认电流、温升和机械行程安全后再提高。`MOTOR_WHEEL_SPEED_LIMIT_MM_S`、`MOTOR_MAX_VX_MM_S`、`MOTOR_MAX_VY_MM_S` 和 `MOTOR_MAX_W_RAD_S` 应以实际稳定可达速度为准，不应用电机空载规格直接填写。

## 2. 单轮前馈、启动补偿和 PID

配置文件：`tm_chassis/include/motor_control.h`；测试位于 `firmware/tests/hardware/`。

| 顺序 | 变量 | 方法 |
| --- | --- | --- |
| 1 | `MOTOR_[A/B/D]_FF_{K,B}_{FWD,REV}` | 运行 `ff-calibration`。它会输出六组 `FF_RESULT`，分别填入 A/B/D 轮正反方向的 `K` 与 `B`。不要把正反方向合并。 |
| 2 | `MOTOR_[A/B/D]_START_PWM_{FWD,REV}`、`MOTOR_STARTUP_*` | 运行 `startup-threshold`，记录每轮正反向首次持续转动的 PWM。默认控制策略未启用启动增强；只有启用 `MOTOR_STARTUP_BOOST_ENABLE` 时才据此调整增强上限和斜率。 |
| 3 | `MOTOR_[A/B/D]_PID_{KP,KI,KD,I_LIMIT}` | 先固定前馈，运行 `small-motion`。先增加 `KP` 到响应足够快但不抖动，再小幅增加 `KI` 消除稳定误差；仅在明显振荡或制动过冲时再增加 `KD`。`I_LIMIT` 用于限制积分累积。 |

`MOTOR_PWM_MIN`、`MOTOR_RUN_PWM_FLOOR_ENABLE` 和旧的 `MOTOR_*_RUN_PWM_*` 主要用于对比或兼容配置。常规标定优先使用前馈和 PID，不要同时叠加多个最低 PWM 策略。

## 3. IMU 和里程计

| 变量组 | 文件 | 方法 |
| --- | --- | --- |
| `MPU6050_I2C_PORT`、`MPU6050_SDA_PIN`、`MPU6050_SCL_PIN`、`MPU6050_I2C_ADDRESS` | `tm_chassis/include/mpu6050.h` | 对照 I2C 接线和模块 AD0 电平；常见地址为 `0x68` 或 `0x69`。 |
| `MPU6050_*CALIBRATION*`、`MPU6050_STATIONARY_*` | 同上 | 运行 `mpu-monitor`。上电后保持车体水平静止，等待校准完成；如果正常静止时频繁校准失败，再检查安装、振动和阈值。 |
| `MPU6050_ACCEL_LPF_ALPHA`、`MPU6050_GYRO_LPF_ALPHA`、`MPU6050_COMPLEMENTARY_ALPHA` | 同上 | 只在确认原始姿态正确后调节。降低滤波系数会更灵敏但更噪；提高互补滤波系数会更依赖陀螺仪。 |
| `CHASSIS_ODOMETRY_DISTANCE_SCALE` | `tm_chassis/include/chassis_odometry.h` | 在轮径已标定后再使用。以 1–2 m 直线手推测试，比例应为 `真实距离 / 里程计距离`。不要同时反复修改轮径和该比例。 |
| `CHASSIS_ODOMETRY_IMU_YAW_SIGN` | 同上 | 手动逆时针旋转底盘，软件 Yaw 应增加；若减少则改为 `-1.0f`。 |

使用 `odometry-push` 时电机输出保持关闭，适合先确认编码器、IMU 和坐标符号；通过后再运行 `small-motion` 验证主动运动。

## 4. 传感器和显示

| 模块 | 变量 | 测量或设置方法 |
| --- | --- | --- |
| 红外 | `IR_SENSOR_L2_PIN`、`IR_SENSOR_L1_PIN`、`IR_SENSOR_R1_PIN`、`IR_SENSOR_R2_PIN` | 按物理顺序从左外、左内、右内、右外填写。确认不与电机、编码器或超声引脚重复；编译期断言会检查电机冲突。 |
| 红外 | `IR_SENSOR_ACTIVE_LEVEL` | 在白底和黑线各读取一次 GPIO。黑线读高设为 `1`，黑线读低设为 `0`。 |
| 超声 | `ULTRASONIC_TRIG_GPIO`、`ULTRASONIC_ECHO_GPIO` | 对照模块接线。Echo 必须满足 ESP32-S3 的电平要求，必要时使用分压或电平转换。 |
| 超声 | `ULTRASONIC_OBSTACLE_CM`、`MIN/MAX_DISTANCE_CM`、`FILTER_SIZE` | 用卷尺在预期停车距离放置平整目标物，连续测量后设置阈值；避障距离应额外包含制动距离和测量波动。`FILTER_SIZE` 必须为正奇数。 |
| OLED | `OLED_I2C_SCL_PIN`、`OLED_I2C_SDA_PIN`、`OLED_ADDR_3C/3D`、`OLED_WIDTH/HEIGHT` | 用 I2C 扫描或模块规格确认地址和引脚；常见地址是 `0x3C` 或 `0x3D`。 |

## 5. 循迹、避障和笔机构

`LINE_KP`、`LINE_KD`、`LINE_V_MAX`、`LINE_V_MIN`、`LINE_LOST_*` 位于 `apps/line-infrared/main/line_tracker.h`。在传感器模式正确、低速能够稳定循迹后再逐步提高速度；出现摆动先降低 `LINE_KP` 或提高 `LINE_KD`，不要先调避障距离。

`AVOID_*_DISTANCE_MM`、`AVOID_*_SPEED_*` 位于 `apps/line-infrared/main/obstacle_avoid.h`。在实际赛道上量出障碍物宽度、传感器到车体前缘的距离、转弯净空和重新找到线的位置后设置。完成每个阶段后都要测试停车与恢复循迹。

笔机构默认参数在 `tm_trajectory/include/pen_control.h`，实际 GPIO 和是否已标定在 `tm_n3/n3_runner_bridge.c` 的 `n3_runner_bridge_default_config()` 中设置。先令 `positions_calibrated=false`，使用安全脉宽范围逐步寻找不顶死机械结构的抬笔和落笔脉宽，再写入 `PEN_CONTROL_DEFAULT_UP_PULSE_US`、`PEN_CONTROL_DEFAULT_DOWN_PULSE_US` 以及移动和稳定时间。未经实际标定，不要把 `positions_calibrated` 设为 `true`。

## 记录模板

每次标定建议在提交说明或维护记录中保存以下内容：

```text
车辆编号：
电池电压 / 载荷 / 地面：
轮径（mm）：
编码器 CPR：A / B / D
底盘半径（mm）：
前馈 FF_RESULT：A+/A-/B+/B-/D+/D-
PID：A / B / D
IMU 地址与方向：
红外黑线电平与引脚：
超声停车阈值（cm）：
验证测试和结果：
```
