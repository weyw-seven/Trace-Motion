# 红外循迹与避障应用

该应用来自原 `test-motor` 任务，使用四路红外传感器进行循迹，并通过超声波与底盘运动模块完成避障。它与 `line-ball-camera` 的目标流程相近，但输入设备与行为逻辑不同：本应用的 `line_tracker` 读取红外模式，后者读取相机视觉结果。

构建前请先审查 `main/infrared_sensor.h` 中的引脚定义。该文件已包含与电机引脚的编译期冲突检查；硬件接线、轮径、PID 和避障距离须与实际车辆一致。

```powershell
idf.py set-target esp32s3
idf.py build
```

本应用已从单头文件实现包整理为独立 ESP-IDF 工程入口，但尚未在当前发布工作区完成实机构建验收。
