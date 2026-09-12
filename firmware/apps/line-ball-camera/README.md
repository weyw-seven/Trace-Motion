# 相机循迹找球应用

该目录包含基于 UVC 相机的循迹、超声避障、找球和推球任务源码。视觉输入与行为状态机与红外循迹应用不同，因此保留为独立应用实现。

| 模块 | 职责 |
| --- | --- |
| `uvc_module`、`image_decode`、`vision_shared_decode` | 相机帧采集和 JPEG 解码 |
| `line_vision`、`colorball_vision` | 黑线和彩球识别 |
| `line_tracker`、`obstacle_avoid` | 基于视觉结果的循迹和避障 |
| `ball_target`、`ball_approach`、`ball_kicker` | 找球、对准、接近与推球 |
| `line_debug_*`、`voice_demo` | 网页调试和语音提示 |

本应用尚未提供完整的 ESP-IDF 根目录、依赖锁定和板级配置。构建前需补齐 UVC、JPEG、HTTP、Wi-Fi、SPIFFS 与语音播放依赖，并将热点配置改为私有构建参数。完成底盘接口适配和实机验证后，可逐步改为复用 `firmware/components/` 中的公共底层组件。
