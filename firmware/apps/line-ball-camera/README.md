# 相机循迹找球应用

这是 Trace Motion 的相机验收任务源码包。它以 UVC 相机为视觉输入，完成视觉循线、超声避障、彩球识别、接近和推球；视觉输入与行为状态机均不同于红外任务，因此保留为独立应用实现。

| 模块 | 职责 |
| --- | --- |
| `uvc_module`、`image_decode`、`vision_shared_decode` | 相机帧采集和 JPEG 解码 |
| `line_vision`、`colorball_vision` | 黑线和彩球识别 |
| `line_tracker`、`obstacle_avoid` | 基于视觉结果的循迹和避障 |
| `ball_target`、`ball_approach`、`ball_kicker` | 找球、对准、接近与推球 |
| `line_debug_*`、`voice_demo` | Wi-Fi 网页调试和语音提示 |

## 当前状态

本目录公开的是任务源码，不是可直接构建的 ESP-IDF 工程：尚缺顶层 `CMakeLists.txt`、组件依赖锁定、`sdkconfig`、板级引脚配置与受控的热点配置。因此仓库不会把它标记为“已可刷写”或“已完成实机验证”。

在补齐工程配置前，源码可用于阅读和继续开发；构建前至少需要确认以下依赖和接口：

1. UVC/USB 视频输入和 JPEG 解码。
2. HTTP 服务、Wi-Fi、SPIFFS 与网页调试资源。
3. 语音播放依赖及其资源文件。
4. 电机、IMU、超声、OLED 和相机的板级引脚与供电配置。
5. 真实热点名称和密码应通过私有构建参数传入，不能写入源码。

`main/main.c` 是当前集成入口，集中定义各模块实现宏。后续将其工程化时，应优先复用 `firmware/components/tm_chassis` 的底盘能力与 `tm_sensors` 的超声能力；视觉、找球、推球和网页调试仍应留在本应用内。完成独立构建和实机验收后，再更新本说明中的构建命令与验证状态。
