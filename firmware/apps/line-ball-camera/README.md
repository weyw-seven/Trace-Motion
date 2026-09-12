# 相机循迹找球应用（实验导入）

该应用实现顺序任务：视觉循迹、超声避障、停车等待、找球与推球。源码保存在 `main/`，按职责分为：

- `uvc_module`、`image_decode`、`vision_shared_decode`：相机帧接收和共享 JPEG 解码。
- `line_vision`、`colorball_vision`：黑线与彩球视觉结果。
- `line_tracker`、`obstacle_avoid`：循迹和超声避障状态机。
- `ball_target`、`ball_approach`、`ball_kicker`：找球、对准、接近和推球行为。
- `line_debug_*`、`voice_demo`：浏览器调试和语音提示。

这是从 task2 文件包原样导入的源码基线，尚缺独立 ESP-IDF 工程的 `CMakeLists.txt`、`idf_component.yml`、`sdkconfig` 和锁定依赖。它依赖 UVC、JPEG 解码、HTTP 服务、Wi-Fi 与 SPIFFS，且当前应用里的 Wi-Fi 默认值只是开发占位符。请先把热点名称和密码改为构建配置，再进行实机验证。

该目录不参与 `apps/trajectory-drawing/` 的构建。补齐依赖、统一底层 API、完成硬件验收和来源确认后，才应把它标记为可发布示例。
