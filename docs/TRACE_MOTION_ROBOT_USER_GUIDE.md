# Trace Motion（追迹）机器人实机使用说明

本说明只覆盖已完成硬件验收后的实机操作。首次接触项目时，应先构建根目录 README 所述的安全档；安全档不会驱动电机或笔机构。

## 刷写固件

在已加载 ESP-IDF 环境的 PowerShell 中，从仓库根目录执行：

```powershell
cd firmware
idf.py set-target esp32s3
idf.py -p COMx flash monitor
```

将 `COMx` 换成实际端口。对新的构建目录，先运行 `idf.py build`。

## Wi-Fi TCP（可选）

Wi-Fi 默认关闭。启用时必须提供自己的热点名称和密码：

```powershell
idf.py -B build-wifi -D N3_ENABLE_WIFI=1 `
  -D N3_WIFI_AP_SSID="你的热点名称" `
  -D N3_WIFI_AP_PASSWORD="至少 8 位的私有密码" build flash
```

ESP32 SoftAP 的默认地址为 `192.168.4.1`，N3 TCP 默认端口为 `5000`。PC 和 ESP32 必须连接同一网络；在 PC 端 UI 的 `Real Robot` 面板选择 Wi-Fi TCP 后填写相应主机和端口。

## USB 串口

在 PC UI 的 `Real Robot` 面板选择 USB Serial，选择 ESP32 对应 COM 口并使用 `115200` 波特率。看到 `HELLO`、`READY` 后再上传任务。

## 执行前检查

1. 确认急停可立即断开运动输出，并清空小车周边。
2. 确认电源、轮子方向、编码器和笔初始位置正确。
3. 连接后先执行 `RESET_POSE`，再上传轨迹。
4. 确认 PC 预检无错误；它只提供提示，固件仍会重新校验任务。
5. 先以低速、短轨迹、轮子悬空或安全场地验证。

运行时如有异常，立即使用 PC UI 的 Stop 或 E-Stop。Wi-Fi 连接中断时，心跳保护应停止任务；恢复连接后应检查状态，并按需要执行 `Clear E-Stop / re-arm`。

默认安全档只允许事件轨迹，不会执行含 `LINE`、`CIRCLE` 或 `CUBIC_BEZIER` 的运动任务。启用真实运动前，必须完成针对当前底盘、供电与机械结构的单独验收。
