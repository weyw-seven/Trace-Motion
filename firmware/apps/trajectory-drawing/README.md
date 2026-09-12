# 轨迹绘图应用

这是 Trace Motion 的创新实验工程。它接收 PC 端生成的 TRJ2 轨迹，并通过 N3 协议完成上传、校验、运行状态回传和受控执行。

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## 默认安全档

默认构建支持连接、上传、格式校验和事件记录，不会驱动电机或笔机构。`N3_ENABLE_HARDWARE`、`N3_ENABLE_MOTION`、`N3_ENABLE_PEN`、`N3_ENABLE_WIFI` 等参数只应在完成对应硬件验收后启用。

| 构建参数 | 默认值 | 作用 |
| --- | --- | --- |
| `N3_ENABLE_HARDWARE` | `0` | 初始化真实硬件。 |
| `N3_ENABLE_MOTION` | `0` | 允许轨迹驱动底盘。 |
| `N3_ENABLE_PEN` | `0` | 允许笔机构事件。 |
| `N3_ENABLE_SIMULATION` | `0` | 启用无硬件轨迹模拟。 |
| `N3_ENABLE_WIFI` | `0` | 启用 SoftAP 与 TCP 端点。 |

例如，先用模拟档验证完整 PC—固件流程：

```powershell
idf.py -B build-sim -D N3_ENABLE_SIMULATION=1 build flash monitor
```

Wi-Fi 必须提供私有热点名称和密码，切勿把实际凭据写入 `CMakeLists.txt` 或提交到仓库。

## 代码和依赖

`main/main.c` 只启动 N3 服务；上传、CRC32 和 TRJ2 校验、串口/TCP 传输、运行桥接和安全停止均由 `tm_n3` 负责。轨迹解码和执行接口位于 `tm_trajectory`，底盘能力位于 `tm_chassis`。这些组件的实现不应复制到本应用目录。

PC 端生成轨迹的方法见 [PC 端说明](../../../pc/README.md)，实机操作流程见 [机器人操作说明](../../../docs/TRACE_MOTION_ROBOT_USER_GUIDE.md)。
