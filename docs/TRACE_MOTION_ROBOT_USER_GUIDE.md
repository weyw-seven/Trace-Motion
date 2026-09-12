# Trace Motion（追迹）实机操作说明

本说明面向已完成硬件接线和基础验收的设备。首次运行时请先构建默认安全档，并在轮子悬空或安全场地进行验证。

## 刷写轨迹绘图应用

在已加载 ESP-IDF 环境的 PowerShell 中运行：

```powershell
cd firmware\apps\trajectory-drawing
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

将 `COMx` 替换为实际端口。默认构建不会驱动电机或笔机构。

## Wi-Fi 连接

Wi-Fi 默认关闭。需要时，在构建阶段传入自己的热点名称和密码：

```powershell
idf.py -B build-wifi -D N3_ENABLE_WIFI=1 `
  -D N3_WIFI_AP_SSID="你的热点名称" `
  -D N3_WIFI_AP_PASSWORD="至少 8 位的私有密码" build flash
```

启用 SoftAP 后，默认地址为 `192.168.4.1`，N3 TCP 默认端口为 `5000`。也可通过 USB Serial 使用 N3 通信。

## 执行前检查

1. 确认急停可以切断运动输出，并清空小车周边区域。
2. 检查电源、电机方向、编码器、MPU6050 和笔机构初始位置。
3. 连接后先重置位姿，再上传短轨迹进行低速验证。
4. 确认 PC 预检无错误；ESP32 仍会重新校验轨迹。
5. 出现异常时立即使用 Stop 或 E-Stop，恢复后重新检查状态。

启用真实运动、笔控制或相机任务前，应对当前底盘、电源与机械结构完成单独验收。
