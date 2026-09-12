# 固件测试

本目录保存独立构建的固件验证程序。它们用于底盘标定、传感器检查和运动验收，不属于机器人任务应用，也不应作为日常运行固件刷写。

目前公开的硬件测试位于 [hardware/README.md](hardware/README.md)。每个测试是独立 ESP-IDF 工程，先进入对应目录并执行：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

测试直接依赖 `firmware/components/tm_chassis`。修改底盘引脚、轮径、编码器或控制参数后，应重新完成受影响测试。
