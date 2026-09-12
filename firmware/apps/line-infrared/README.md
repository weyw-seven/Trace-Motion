# 红外循迹与避障应用

该应用使用四路红外传感器识别线条，通过超声传感器检测障碍物，并调用三轮全向底盘控制完成循迹和避障。

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

应用链接 `tm_chassis`、`tm_sensors` 和 `tm_line_ir`。它不复刻公共实现；引脚定义、轮径、PID、红外判定和避障距离均在对应组件中维护。刷写前应按实际车辆完成接线检查和低速台架验证。
