# 红外循线与超声避障

这是 Trace Motion（追迹）的红外验收任务工程。它使用四路红外传感器跟随赛道，使用超声波检测正前方障碍物，并通过三轮全向底盘完成横移绕障；OLED 显示当前运行状态和传感器数据。

本目录是一个可以独立构建、阅读和调参的 ESP-IDF 应用。红外循线的任务逻辑不隐藏在公共组件中：从 `main/` 目录即可找到启动过程、循线控制、避障状态机和显示逻辑。

## 代码布局

| 文件 | 作用 |
| --- | --- |
| `main/main.c` | 应用入口，按依赖顺序初始化底盘、IMU、里程计、循线、超声、避障和 OLED。 |
| `main/line_infrared_app.c` | 本应用的实现单元；在此处唯一启用循线、避障和 OLED 模块实现。 |
| `main/line_tracker.h` | 四路红外模式解析、加权偏差、滤波 PD、自适应速度、起始搜线、丢线恢复和终点确认。 |
| `main/obstacle_avoid.h` | 超声触发、循线控制权交接、横移--前进--归线的绕障状态机。 |
| `main/oled_display.h` | SSD1306 OLED 初始化与状态显示。 |
| `../../components/tm_chassis/` | 共享的电机闭环、MPU6050、三轮运动学、里程计和运动基元。 |
| `../../components/tm_sensors/` | 共享的红外 GPIO 与超声 RMT 驱动。 |

`main/*.h` 使用单头文件实现形式，实际代码只会由 `line_infrared_app.c` 编译一次。不要在其他 `.c` 文件中定义 `LINE_TRACKER_IMPLEMENTATION`、`OBSTACLE_AVOID_IMPLEMENTATION` 或 `OLED_DISPLAY_IMPLEMENTATION`，否则会产生重复符号。

## 运行逻辑

1. `main.c` 初始化电机、IMU、里程计和底盘运动控制。
2. 循线模块读取四路红外模式，将其转换为连续横向偏差，并用带低通微分项的 PD 控制转向。
3. 前进速度随偏差和转向强度降低；发生丢线时，状态机根据最后一次观测方向搜索并恢复线路。
4. 超声模块检测到障碍物后，避障状态机暂停循线输出，调用底盘运动基元完成横移、前进和回归；重新发现线路后恢复循线。
5. OLED 周期性显示传感器和运行状态，便于实机调试。

## 构建与刷写

需要 ESP-IDF 5.4.4。加载 ESP-IDF 环境后，在本目录执行：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

首次地面测试前，应使驱动轮悬空或清空周边区域，并确认电机方向与 Stop 行为正确。

## 实车配置

| 配置类别 | 修改位置 |
| --- | --- |
| 电机引脚、编码器、轮径、前馈和速度 PID | `../../components/tm_chassis/include/motor_control.h` |
| IMU、里程计和底盘运动参数 | `../../components/tm_chassis/include/mpu6050.h`、`chassis_odometry.h`、`chassis_motion.h` |
| 红外与超声引脚、黑线有效电平、超声阈值 | `../../components/tm_sensors/include/infrared_sensor.h`、`ultrasonic_sensor.h` |
| 循线 PD、速度和丢线恢复参数 | `main/line_tracker.h` |
| 绕障距离、速度与状态机参数 | `main/obstacle_avoid.h` |
| OLED I2C 引脚和地址 | `main/oled_display.h` |

具体测量步骤和验证方法见 [硬件配置与标定指南](../../../docs/HARDWARE_CALIBRATION.md)。建议先完成底盘标定和低速循线，再逐步提高 `LINE_V_MAX` 并调整绕障距离。
