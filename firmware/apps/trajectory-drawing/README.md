# 绘图机器人固件

这是 Trace Motion 绘图机器人的 ESP-IDF 工程根目录。默认构建为安全档，仅允许连接、上传、校验与事件轨迹执行，不会驱动电机或笔机构。

```powershell
idf.py set-target esp32s3
idf.py build
```

完整使用方式见仓库根目录 README 与 `docs/TRACE_MOTION_ROBOT_USER_GUIDE.md`。
