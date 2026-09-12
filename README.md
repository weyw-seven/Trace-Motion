# N3 绘图机器人

N3 是一个三轮全向底盘绘图机器人项目。PC 端将图片、手绘路径或虚拟地图中的路径编译为 TRJ2 轨迹；ESP32-S3 固件验证、存储并执行轨迹，同时提供 USB 串口和可选的 Wi-Fi TCP 通信。

> 当前默认固件是安全档：可以连接、上传、校验和执行 `PEN_UP`、`PEN_DOWN`、`WAIT` 事件，但默认不会使电机或笔机构动作。真实运动必须在构建时显式开启，并先完成硬件验收。

## 仓库结构

```text
pc/        Python 轨迹规划、TRJ2 编解码、预览、地图 UI 与 PC—ESP32 通信
firmware/  ESP-IDF / ESP32-S3 N3 服务、轨迹校验与运动执行代码
docs/      架构和实机使用说明
archive/   历史实验代码与开发交接记录；不参与正式构建
```

PC 决定路径内容，ESP32 负责最终的安全校验和实时执行。TRJ2 是两端共享的二进制轨迹格式；相关架构见 [docs/N3_ARCHITECTURE.md](docs/N3_ARCHITECTURE.md)。

## PC 端快速开始

需要 Python 3.10 以上。下面以 PowerShell 为例：

```powershell
cd pc
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\python.exe -m pytest -ra
```

将图片转为 TRJ2 轨迹：

```powershell
.\.venv\Scripts\python.exe -m pc_trajectory.cli lineart graph\image.png `
  --output output --max-extent-mm 100 --threshold 127 `
  --min-component-pixels 80 --simplify-mm 0.35
```

常用可选依赖：`.[raster]` 用于图像线稿处理，`.[demo]` 用于相机和地图演示，`.[serial]` 用于 USB 串口连接。

## ESP32 固件快速开始

安装 ESP-IDF 5.4.4，打开已加载 ESP-IDF 环境的 PowerShell：

```powershell
cd firmware
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

默认构建不会初始化硬件运动。用于无硬件验证的仿真档可显式配置：

```powershell
idf.py -B build-sim -D N3_ENABLE_SIMULATION=1 build
```

只有在完成急停、电源、方向和空载验收后，才考虑开启 `N3_ENABLE_HARDWARE`、`N3_ENABLE_MOTION` 与笔机构相关选项。构建参数说明见 [firmware/main/n3_build_config.h](firmware/main/n3_build_config.h)。

若启用 Wi-Fi，请在构建命令中传入自己的热点名称和强密码；仓库中的默认密码只是占位符，不能用于公开或实机部署：

```powershell
idf.py -B build-wifi -D N3_ENABLE_WIFI=1 `
  -D N3_WIFI_AP_SSID="你的热点名称" `
  -D N3_WIFI_AP_PASSWORD="至少 8 位的私有密码" build
```

## 通信与安全

PC 与固件通过 N3 JSON 控制帧和 TRJ2 原始字节交互。固件会再次验证长度、CRC32、TRJ2 格式和当前构建能力；PC 端预检仅用于尽早提示，不能替代固件的安全检查。

实机运行前请阅读 [docs/N3_ROBOT_USER_GUIDE.md](docs/N3_ROBOT_USER_GUIDE.md)，确认急停可用、轮子悬空或周边清空，并从安全档开始验证。

## 归档内容

`archive/firmware-legacy-experiments/` 存放历史的电机、传感器和轨迹试验入口，以及已被替代的头文件版本。它们保留作参考，不被 `firmware/main/CMakeLists.txt` 编译。`archive/development-notes/` 保存阶段性交接和设计记录，其中的计划与状态不一定代表当前发布版本。

## 许可证

本仓库在添加 `LICENSE` 前尚未向外授予使用许可。建议采用 Apache License 2.0；理由、适用条件和添加步骤见 [LICENSE-RECOMMENDATION.md](LICENSE-RECOMMENDATION.md)。
