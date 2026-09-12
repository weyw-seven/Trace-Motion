# 轨迹绘图应用

该 ESP-IDF 应用接收 PC 端生成的 TRJ2 轨迹，并通过 N3 协议完成上传、校验、运行状态回传和受控执行。

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

默认构建为安全档：支持连接、上传、校验和事件记录，不会驱动电机或笔机构。`N3_ENABLE_HARDWARE`、`N3_ENABLE_MOTION`、`N3_ENABLE_PEN`、`N3_ENABLE_WIFI` 等参数只应在完成对应硬件验收后启用。

应用依赖 `tm_n3`，后者再引用底盘和轨迹公共组件。请不要在本目录重复加入 `n3_*`、`trajectory_*` 或 `chassis_*` 的实现文件。
