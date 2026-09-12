# Trace Motion PC 端

本目录提供 Trace Motion 的桌面端工具：将线稿或手绘轨迹转换为 TRJ2 文件，预览笔迹与运动路径，并通过 USB Serial 或 Wi-Fi TCP 与 ESP32 的 N3 服务通信。

## 功能入口

| 命令 | 用途 | 可选依赖 |
| --- | --- | --- |
| `pc-trajectory lineart` | 命令行将 PNG/JPEG 线稿编译为 TRJ2，并生成预览和报告。 | `raster` |
| `python -m pc_trajectory.ui` | 线稿转换桌面界面。 | `raster` |
| `pc-trajectory-map` | 地图、路径规划、轨迹审查与机器人连接界面。 | `demo`、`serial` |
| `pc-trajectory-calibrate` | 相机标定界面。 | `demo` |

## 安装与验证

需要 Python 3.10 或更高版本。在本目录执行：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev,demo,serial]"
.\.venv\Scripts\python.exe -m pytest -ra
```

`.[dev]` 覆盖核心线稿处理和测试依赖；`demo` 用于地图与相机相关界面；`serial` 用于 USB 串口连接。

## 线稿转换示例

```powershell
pc-trajectory lineart graph\image.png --output output `
  --threshold 127 --min-component-pixels 80 `
  --max-extent-mm 100 --simplify-mm 0.35
```

输出目录包含 TRJ2 文件、处理阶段图、轨迹预览和 JSON 报告。生成成功只表示格式和几何预检通过；实际运行前仍需在 ESP32 端完成硬件检查和低速验证。

## 代码布局

| 目录或模块 | 职责 |
| --- | --- |
| `raster/` | 图像预处理、骨架提取、笔画排序、曲线拟合和线稿编译。 |
| `drawing.py`、`geometry.py`、`toolpath*.py` | 几何模型、绘图记录与工具路径编译。 |
| `traj*_*.py` | TRJ1/TRJ2 的读写、导入、格式校验和兼容处理。 |
| `ui.py` | 线稿到 TRJ2 的桌面界面。 |
| `navigation_ui.py`、`demo/` | 地图导航、相机标定、轨迹审查、模拟器与真实机器人连接。 |
| `tests/` | 单元和端到端格式验证测试。 |
| `examples/` | 生成验收轨迹、预览和联调夹具的脚本。 |

协议、固件安全边界和实机操作见仓库根目录 [README](../README.md) 与 [机器人操作说明](../docs/TRACE_MOTION_ROBOT_USER_GUIDE.md)。
