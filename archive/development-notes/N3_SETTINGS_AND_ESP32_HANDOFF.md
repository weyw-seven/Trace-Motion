# N3 实车执行与地图设置持久化交接文档

本文用于把当前 PC 虚拟地图项目交接给下一位 agent。目标分成两条线：

1. 保存 `.vmap.json` 时同步保存规划、车体和导航设置；
2. 把 PC UI 中已经验证过的 TRJ2 任务，通过 USB/Wi-Fi N3 协议交给 ESP32-S3 的底盘、里程计和 Pen 执行。

## 0. 给下一位 agent 的启动指令

先阅读本文，然后按“阶段 A：设置持久化”开始工作。第一阶段只修改
`D:\esp-projects\pc_trajectory`，不要假设 ESP32 已经收到 HELLO，也不要在
没有实车急停保护时触发电机运动。设置持久化和测试通过后，再进入 ESP32
工程实现 N3 空载服务。每个阶段都要保留可运行的 PC loopback 验收，并在交接
时记录测试命令、生成文件和未完成事项。

## 1. 当前工作区和代码状态

PC 工程：

```text
D:\esp-projects\pc_trajectory
```

ESP32 工程：

```text
D:\esp-projects\test-motor
```

PC 端已经具备：

- 虚拟地图编辑器：路径、Polyline、Freehand、Home、Landmark、障碍物；
- Map→WORLD 坐标转换，约定为 `+X 前进、+Y 左、正 yaw 逆时针`；
- 车体外形安全范围和障碍物膨胀；
- 路径避障、路径顺序选择、Pen Up/Pen Down 预览；
- Exact Polyline 与 Smooth Freehand 两种路径模式；
- LINE+CIRCLE 拟合、平滑、简化和 TRJ2 预检面板；
- `SerialRobotLink` 和 `TcpRobotLink`；
- PC 端 N3 JSON/二进制上传协议和 loopback 测试；
- 严格 TRJ2 V2 编码、解码、CRC/尺寸/连续性检查。

最近一次 `cheap_map` 验收结果位于：

```text
test_results/virtual_map_demo/cheap_map_traj_review/
```

当前验收文件的主要结果：

- TRJ2 V2；
- 32-byte header，44-byte record；
- 17 records，780 bytes；
- 12 LINE，0 CIRCLE，3 PEN_UP，2 PEN_DOWN；
- CRC32：`66828300`；
- 已通过严格 PC 解码和膨胀障碍物碰撞检查。

### 1.1 2026-09-08 离线开发进度

在没有实车和 ESP32 的阶段，ESP32 工程已经继续推进到“可上传、不可运动”的安全版本：

- `D:\esp-projects\test-motor\main\n3_service.c/h` 已提供 N3 HELLO、STATUS、POSE、PING、STOP、ESTOP、CLEAR_ESTOP、RESET_POSE；
- HELLO 启动后每秒补发一次，允许 PC 在 ESP32 已经运行后才打开串口；
- `UPLOAD_BEGIN` 后切换为定长二进制接收，写入 SPIFFS 根目录临时文件 `/spiffs/n3_upload.tmp`；
- `UPLOAD_END` 会检查声明长度、实际长度、CRC32，并逐条调用 TRJ2 decoder；验证成功后才替换 `/spiffs/n3_current.traj`；
- 上传中断超过 5 秒会删除临时文件并回到 IDLE，避免断线后一直卡在 UPLOADING；
- STATUS 会回报已确认的 `current_job`，失败会回报错误文本；
- `ROTATE_REL` 仍明确返回 `MOTOR_NOT_READY`，不会产生运动；`RUN_TRAJECTORY` 已接入
  安全的 event-only runner：只执行 `PEN_UP`、`PEN_DOWN`、`WAIT` 的逻辑状态，含
  `LINE`/`CIRCLE`/`CUBIC_BEZIER` 的任务同步返回 `MOTION_DISABLED`；
- 电机底层急停锁存已修正：普通 `motor_stop()`、`Set_motor()`、`motor_set_velocity()` 和 raw PWM 调试调用不再清除急停，只有显式 `motor_control_enable(true)` 才能重新使能；
- N3 ACK 缓存现在按“命令 ID + 原始请求指纹”命中，避免 PC 重连后 ID 从 1 重新开始时误回旧 ACK；
- JSON 和换行符保持单次原子传输，避免 UI 的 `extra data invalid JSON`。
- 生产入口已经从旧底盘回归测试中拆出：`main.c` 只启动 N3 service；
- 底盘/MPU/运动模块的实现宏集中在 `robot_stack_impl.c`，TRJ2 decoder 的实现宏集中在 `trajectory_stack_impl.c`；
- `n3_build_config.h` 建立了编译期安全档位，默认 `build_profile=safe`、`hardware_enabled=false`、`motion_enabled=false`、`pen_enabled=false`；
- HELLO/STATUS 会报告上述构建能力，防止把安全固件误认为可运动固件；
- ESP-IDF 5.4.4 完成重新配置和完整链接，生成新的 `build/test-motor.bin`。

当前安全固件构建产物：

```text
D:\esp-projects\test-motor\build\test-motor.bin
D:\esp-projects\test-motor\build\storage.bin
D:\esp-projects\test-motor\build\bootloader\bootloader.bin
D:\esp-projects\test-motor\build\partition_table\partition-table.bin
```

离线已通过 `tests/test_robot_link.py`、`tests/test_n0_protocol.py`；完整 PC 测试还受到本机 `C:\Users\jerry\AppData\Local\Temp\pytest-of-jerry` 权限限制，需在可写临时目录重新运行。

有板后的无运动验收脚本为 `examples/accept_n3_safe_firmware.py`。它会先确认
HELLO 的 `build_profile=safe` 且硬件、运动、Pen 全部为 false，然后验证 READY、PING、上传、CRC/decoder 结果、STATUS 的 `current_job`、事件轨迹完成、运动轨迹被 `MOTION_DISABLED` 拒绝、RESET_POSE、STOP、ESTOP、CLEAR_ESTOP。

### 1.2 2026-09-09 D2：安全 event-only runner

当前安全固件已增加 `main/n3_runner_bridge.c/h`。它在独立 FreeRTOS task 中重新打开
已上传的 TRJ2 文件，预检全部记录后只允许三类事件：

- `PEN_UP` / `PEN_DOWN`：只更新逻辑 Pen 状态，不调用物理 Pen 驱动；
- `WAIT`：分片等待并持续检查 STOP/ESTOP；
- `LINE`、`CIRCLE`、`CUBIC_BEZIER`：返回 `MOTION_DISABLED`，不创建执行任务。

STATUS 新增 `execution_mode=EVENT_ONLY`、`record_index`、`record_count`，并回报
`runner=STARTING/RUNNING/FINISHED/STOPPED/ESTOPPED/ERROR`。事件 runner 的总 WAIT
时间上限为 5 分钟；STOP/ESTOP 都是协作式中断，ESTOP 还会锁存到 CLEAR_ESTOP。

PC 侧新增：

- `examples/generate_n3_event_runner_fixtures.py`：生成可重复的完成、长等待、运动拒绝三种 fixture；
- `examples/accept_n3_event_runner.py`：一次覆盖 FINISHED、STOPPED、ESTOPPED/CLEAR、MOTION_DISABLED；
- `D:\esp-projects\test-motor\spiffs\n3_event_done.traj`；
- `D:\esp-projects\test-motor\spiffs\n3_event_wait.traj`；
- `D:\esp-projects\test-motor\spiffs\n3_event_motion.traj`。

已用 ESP-IDF 5.4.4 完整构建安全镜像；下一步上板时先刷入新的
`D:\esp-projects\test-motor\build\test-motor.bin` 和包含上述 fixture 的
`storage.bin`，再执行 D2 验收脚本。此阶段仍不接入电机、里程计、tracker 或物理 Pen。

D2 上板验收命令：

```powershell
cd D:\esp-projects\test-motor
idf.py -p COM9 flash

cd D:\esp-projects\pc_trajectory
.\.conda\python.exe -s examples\accept_n3_event_runner.py `
  --port COM9 `
  --complete-traj D:\esp-projects\test-motor\spiffs\n3_event_done.traj `
  --long-wait-traj D:\esp-projects\test-motor\spiffs\n3_event_wait.traj `
  --motion-traj D:\esp-projects\test-motor\spiffs\n3_event_motion.traj
```

通过标准依次为：`READY safe event-only profile`、`EVENT FINISHED`、
`STOP -> STOPPED`、`ESTOP -> ESTOPPED`、`CLEAR_ESTOP -> IDLE`、
`motion trajectory rejected with MOTION_DISABLED`，最后输出
`N3 EVENT RUNNER ACCEPTANCE PASSED`。脚本上传轨迹后才执行，fixture 不要求预先
存在 ESP32 文件系统；`idf.py flash` 会同时刷入当前构建的 app、partition 和 storage。

### 1.3 2026-09-09 D3：硬件无关的 simulation profile（不含 CUBIC）

在没有实车和 ESP32 的阶段，已增加独立的 `simulation` 编译档位。它不初始化电机、
里程计或物理 Pen，但在 ESP32 上执行与未来运动路径共用的 `trajectory_executor`：

- `LINE`、`CIRCLE`：使用 TRJ2 几何、速度/加速度规划和连续段 look-ahead，更新合成 POSE；
- `PEN_UP`、`PEN_DOWN`、`WAIT`：更新逻辑状态并参与完整 runner 状态机；
- `STOP`、`ESTOP`、`CLEAR_ESTOP`、`RESET_POSE`：与 safe runner 相同的协作式安全语义；
- 运行中再次上传：返回 `RUNNER_BUSY`；
- `CUBIC_BEZIER`：明确返回 `UNSUPPORTED_RECORD`，本阶段不实现贝塞尔；
- HELLO/STATUS/POSE 会报告 `build_profile=simulation`、`execution_mode=SIMULATED`、
  `simulation_enabled=true`，并保持 `hardware_enabled=false`、`motion_enabled=false`、
  `pen_enabled=false`。

ESP32 工程新增：

- `main/n3_build_config.h`：增加 `N3_ENABLE_SIMULATION`，并禁止 simulation 与真实硬件/运动档位同时打开；
- `main/n3_runner_bridge.c/h`：增加 simulation runner、合成 POSE 和状态字段；
- `main/trajectory_stack_impl.c`：将无硬件的 `trajectory_executor` 纳入公共实现；
- `main/n3_service.c`：报告 simulation 能力并把 RESET_POSE 转发给 runner。

PC 侧新增：

- `examples/generate_n3_simulation_fixtures.py`；
- `examples/accept_n3_simulation.py`；
- `D:\esp-projects\test-motor\spiffs\n3_sim_mixed.traj`（LINE+CIRCLE+PEN+WAIT）；
- `D:\esp-projects\test-motor\spiffs\n3_sim_wait.traj`、`n3_sim_line.traj`、`n3_sim_cubic.traj`。

独立模拟镜像已经生成：

```text
D:\esp-projects\test-motor\build-sim\test-motor.bin
D:\esp-projects\test-motor\build-sim\storage.bin
D:\esp-projects\test-motor\build-sim\bootloader\bootloader.bin
D:\esp-projects\test-motor\build-sim\partition_table\partition-table.bin
```

离线结果：ESP-IDF 5.4.4 的 safe 镜像和 simulation 镜像均完成链接；PC 全量测试在工程内
可写临时目录下通过。simulation 镜像随后已在 COM9 实际刷写并通过下面的完整串口验收。

D3 上板验收命令：

```powershell
cd D:\esp-projects\test-motor
idf.py -B build-sim -p COM9 flash

cd D:\esp-projects\pc_trajectory
.\.conda\python.exe -s examples\accept_n3_simulation.py `
  --port COM9 `
  --mixed-traj D:\esp-projects\test-motor\spiffs\n3_sim_mixed.traj `
  --wait-traj D:\esp-projects\test-motor\spiffs\n3_sim_wait.traj `
  --line-traj D:\esp-projects\test-motor\spiffs\n3_sim_line.traj `
  --cubic-traj D:\esp-projects\test-motor\spiffs\n3_sim_cubic.traj
```

预期依次看到：`PASS READY hardware-free simulation profile`、
`PASS LINE+CIRCLE+PEN+WAIT FINISHED` 且末端 POSE 约为 `(0, 100)` mm、
`PASS STOP -> STOPPED`、`PASS ESTOP -> ESTOPPED`、`PASS CLEAR_ESTOP -> IDLE`、
`PASS upload while running rejected with RUNNER_BUSY`、
`PASS CUBIC rejected with UNSUPPORTED_RECORD`、`PASS RESET_POSE`，最后输出
`N3 SIMULATION ACCEPTANCE PASSED`。整个测试不应有电机或物理 Pen 动作；它只验证协议、文件、
轨迹执行器和状态机，为后续真实 motion profile 上板减少联调范围。

D3 首次串口验收中发现并修复了一个预检分支错误：simulation 已接受 `LINE/CIRCLE`
后错误落入 event-only 检查，导致合法运动轨迹返回 `TRJ_INVALID`。修复后重新生成的
`build-sim/test-motor.bin` 已完成链接；重新刷写后应进入真正的 LINE/CIRCLE simulation
执行阶段。

D3 后续串口验收又定位到 PC 端上传事务的并发缺陷：后台 heartbeat 可能在
`UPLOAD_BEGIN` 与轨迹二进制之间插入 `PING`，ESP32 会按已声明的二进制长度把该 JSON
误当成轨迹数据，因此故障会随机表现为 `CRC_MISMATCH` 或 `UPLOAD_END` ACK 超时。
`SerialRobotLink.upload()` 现已用同一把可重入写锁覆盖 BEGIN、二进制和 END 全过程，
并新增 heartbeat 竞争回归测试。PC 全量测试结果为 `674 passed`。

2026-09-09 在 COM9 上重跑 D3：上述八项检查全部 PASS，最终输出
`N3 SIMULATION ACCEPTANCE PASSED`；其中末端轨迹 POSE 为 `(0, 100, 0)`，
RESET_POSE 为 `(120, -40, 15)`。该并发修复仅修改 PC 端，无需重新刷写 ESP32。

### 1.5 2026-09-09：M0/M1 hardware-check 档位已实现

ESP32 工程现在将正式 N3 入口固定为 `main/n3_app.c`，不再使用容易被手工
电机实验覆盖的 `main/main.c`。原有 `main.c` 保留为手工小速度电机测试源码。

新增 `N3_ENABLE_HARDWARE=1, N3_ENABLE_MOTION=0` 的 `hardware-check` 档位：

- 启动后异步初始化 `motor_control`、MPU6050 和 `chassis_odometry`；
- HELLO 不等待硬件初始化，STATUS 报告 `hardware_initializing`、
  `motor_ready`、`odom_ready` 和初始化错误；
- POSE 来自真实里程计；
- `RESET_POSE` 调用真实 `chassis_odometry_reset()`；
- `ROTATE_REL` 和运动轨迹均返回 `MOTION_DISABLED`；
- STOP/ESTOP/CLEAR_ESTOP 已接入硬件安全动作，但仍没有轨迹运动后端。

对应 PC 验收脚本为 `examples/accept_n3_hardware_check.py`。本档位固件已完成
本地 ESP-IDF 编译链接，输出位于 `D:\esp-projects\test-motor\build-hw-check`；
尚未刷写实车，等待按验收步骤进行上板检查。

实车首次验收发现 MPU6050 启动失败后，初始化顺序已与被动里程计实测程序
对齐：电机编码器初始化后先将 H 桥 STBY 关闭，保持底盘水平静止完成 MPU6050
校准和里程计启动，再以零速度恢复电机驱动。对可恢复的“未静止/未水平”校准
错误自动重试一次；失败时 STATUS 现在包含具体 ESP-IDF 错误名，PC 验收脚本也会
立即报告该错误，不再等待 20 秒后只给出通用超时。

实车复验已通过 READY、电机/里程计就绪、RESET_POSE、ROTATE_REL 拒绝和运动
轨迹拒绝。随后发现验收脚本会把预期运动拒绝留下的 runner `error` 误认为硬件
初始化错误，并在等待 ESTOP 状态时提前退出。现已将硬件错误快速失败限定在最初
的 motor/odometry ready 阶段；ESTOP/CLEAR_ESTOP 等后续状态等待允许保留上一条
命令的诊断信息。该修复仅涉及 PC 脚本，不需要重新刷写 ESP32。

### 1.4 2026-09-09：已取得实车，验收主线切换到 ESP32 实机

现在已经取得实车。后续验收以 ESP32 串口实测为准；D3 simulation 仍保留为
代码回归和没有车辆时的辅助工具，但不再作为最终功能验收依据。

当前必须明确区分三个档位：

| 档位 | 当前用途 | 是否允许车轮运动 |
|---|---|---:|
| `safe` | 协议、上传、事件、急停回归 | 否 |
| `simulation` | 无硬件 LINE/CIRCLE/POSE 状态机回归 | 否 |
| `motion` | 下一阶段接入真实底盘、里程计和 Pen | 暂未验收 |

虽然 `N3_ENABLE_MOTION` 编译开关已经存在，真实 `motion` runner 尚未接入
`trajectory_runner`、`trajectory_tracker`、`chassis_motion` 和
`chassis_odometry`，因此当前不能仅把开关改成 1 就上车。下一步开发顺序为：

1. 新增硬件 motion runner，复用已有 `trajectory_runner`/`trajectory_tracker`，并把
   STATUS/POSE 的 `motor_ready`、`odom_ready`、`tracker` 和错误状态接入 N3；
2. 首先只启用底盘、不启用物理 Pen，车辆架空验证 `STOP`、`ESTOP`、低速
   `ROTATE_REL` 和正负角方向；
3. 架空通过后落地执行一条低速、100 mm 的纯 `LINE`，检查实测 POSE、停止距离和
   轨迹终点；
4. 再验证 `CIRCLE`、连续 LINE+CIRCLE 和轨迹中断/恢复边界；
5. 最后单独打开 `N3_ENABLE_PEN=1`，验证 PEN_UP/PEN_DOWN、急停时自动抬笔和
   完整地图路径。

实车阶段每次刷写前都要先确认 HELLO 的 `build_profile` 和能力字段。若仍是
`safe`/`simulation`，只能做无运动验收；若是 `motion`，必须确认车轮架空、急停
可触达、遥控断电路径可用后才能发送运动命令。CUBIC_BEZIER 仍保持未实现，直到
PC 端正式输出该记录类型。

## 2. 第一条工作线：保存地图时保存其他设置

### 2.1 当前缺口

`pc_trajectory/demo/map_model.py` 的 `MapDocument.to_dict()` 目前只保存：

- workspace 尺寸；
- view；
- Home；
- background；
- paths；
- landmarks；
- obstacles。

下面这些设置目前只存在 `navigation_ui.py` 的内存变量或 `PlannerConfig` 中：

- `travel_speed_mm_s`、`draw_speed_mm_s`、`acceleration_mm_s2`；
- `enable_arc_fitting`、`geometry_tolerance_mm`；
- `min_arc_points`、`min_arc_sweep_deg`、`min_arc_radius_mm`；
- `smoothing_iterations`、`smoothing_strength`、`simplify_tolerance_mm`；
- `obstacle_clearance_mm`、`obstacle_sample_step_mm`；
- `VehicleProfile` 六个尺寸/安全参数；
- nav mode、Pen mode、direction；
- 当前目标点和已选路径顺序；
- 最近一次 USB/Wi-Fi 连接参数。

### 2.2 推荐 JSON 结构

在 `.vmap.json` 根对象增加一个可选的 `settings` 区块，保持旧地图兼容：

```json
{
  "settings": {
    "schema": 1,
    "planner": {
      "travel_speed_mm_s": 100.0,
      "draw_speed_mm_s": 65.0,
      "acceleration_mm_s2": 0.0,
      "geometry_tolerance_mm": 2.0,
      "enable_arc_fitting": true,
      "min_arc_points": 6,
      "min_arc_sweep_deg": 12.0,
      "min_arc_radius_mm": 1.0,
      "smoothing_iterations": 2,
      "smoothing_strength": 0.35,
      "simplify_tolerance_mm": 0.5,
      "continuity_tolerance_mm": 0.000001,
      "obstacle_clearance_mm": 2.0,
      "obstacle_sample_step_mm": 5.0
    },
    "vehicle": {
      "enabled": false,
      "front_mm": 70.0,
      "rear_mm": 70.0,
      "left_mm": 60.0,
      "right_mm": 60.0,
      "safety_margin_mm": 5.0,
      "localization_margin_mm": 0.0
    },
    "navigation": {
      "mode": "click",
      "pen_mode": "move",
      "direction": "auto",
      "selected_path_ids": [],
      "target_map": null
    },
    "transport": {
      "type": "serial",
      "port": "",
      "baudrate": 115200,
      "host": "",
      "tcp_port": 5000
    }
  }
}
```

`transport` 只保存上次使用的地址，不保存连接状态、运行状态或 E-Stop 状态。加载地图后必须仍然由用户点击 Connect。

### 2.3 推荐实现顺序

1. 在 `MapDocument` 增加可选的 `settings` 字段，旧文件没有该字段时使用空设置；
2. 增加设置解析和类型/范围校验；
3. 在 UI 保存前把当前控件值组装成 settings 快照；
4. 在 UI 打开地图后恢复 PlannerConfig、VehicleProfile、导航选择和连接参数；
5. `Save` 保存到当前文件，`Save as...` 创建新文件；
6. 地图内容改变时仍然让已有 `_plan` 失效，防止使用旧 CRC 上传；
7. 未知设置字段忽略，未知 schema 降级到默认值并给出提示。

建议不要让 `map_model.py` 直接 import `PlannerConfig`，避免循环依赖。可以使用独立的纯数据 `MapProjectSettings`，或者先用经过 JSON 校验的字典，再由 UI 转成 `PlannerConfig`。

### 2.4 设置持久化验收

1. 修改拟合参数、速度、车体尺寸、导航模式、Pen 模式、方向和路径顺序；
2. 保存地图并关闭 UI；
3. 重新打开地图；
4. 检查所有面板值恢复；
5. 重新 Plan，检查 TRJ2 的 record count、LINE/CIRCLE 数量和 CRC 与保存前一致；
6. 打开一个旧版没有 `settings` 的 `.vmap.json`，确认仍能正常加载；
7. 手动改坏一个设置值，确认只回退到默认值，不破坏地图几何。

## 3. 第二条工作线：从 UI 落实到 ESP32 执行

### 3.1 当前真实状态

PC 侧 UI 连接流程位于 `pc_trajectory/navigation_ui.py`：

```text
Plan job
  -> NavigationPlanner
  -> NavigationPlan.trj2_bytes()
  -> SerialRobotLink.upload()
  -> SerialRobotLink.run()
```

`pc_trajectory/demo/robot_link.py` 已经实现：

- USB 串口和 TCP 两种传输；
- 后台读取线程；
- 非 JSON ESP-IDF 启动日志作为诊断信息；
- HELLO 握手；
- ACK/ERROR 等待；
- PING 心跳；
- `UPLOAD_BEGIN`、原始二进制、`UPLOAD_END`；
- CRC32；
- STOP、ESTOP、CLEAR_ESTOP、RESET_POSE、ROTATE_REL、RUN_TRAJECTORY。

ESP32 工程保留了原有底盘运动测试代码，但当前 `app_main()` 只启动安全的 N3 service；底盘、Pen 和 runner 尚未接入 N3 执行路径。因此，COM 口打开不等于 UI 已经具备实车执行能力；当前固件只允许通信和上传，不允许真实运动。

### 3.2 ESP32 端应新增的层

当前实现：

```text
D:\esp-projects\test-motor\main\n3_service.h
D:\esp-projects\test-motor\main\n3_service.c
```

服务层职责：

1. 初始化 SPIFFS 和 N3 串口/USB 传输；
2. 在启动后发送 HELLO；
3. 以换行 JSON 解析命令；
4. 收到 `UPLOAD_BEGIN` 后切换到定长二进制接收模式；
5. 把 payload 写入 SPIFFS 根目录临时文件 `/spiffs/n3_upload.tmp`；
6. 在 `UPLOAD_END` 校验 size 和 CRC32；
7. 调用 `trajectory_decoder_open()` 做 TRJ2 校验；
8. `RUN_TRAJECTORY` 交给 `n3_runner_bridge` 做 event-only 预检和执行；
9. 周期性上报 STATUS 和 POSE；
10. 对 STOP/ESTOP 做协议层状态更新，并让 runner 任务协作式退出；底层电机急停锁存已单独修正，待后续硬件 runner 接入时使用。

`trajectory_decoder_open()` 的接口接收文件路径，因此上传不能只停留在临时 JSON 缓冲区中，必须落到 ESP32 可读的文件系统，或者后续为 decoder 增加内存流接口。第一版建议直接使用 LittleFS/SPIFFS 文件。

### 3.3 N3 协议最小实现

所有 JSON 消息都是 UTF-8、单行、以 `\n` 结尾。原始 `.traj` 字节只在 `UPLOAD_BEGIN` 和 `UPLOAD_END` 之间出现。

启动握手：

```json
{"type":"HELLO","protocol":1,"firmware":"n3-esp32","features":["upload","pose","status"]}
```

状态：

```json
{
  "type":"STATUS",
  "runner":"IDLE",
  "tracker":"IDLE",
  "pen":"UP",
  "motor_ready":true,
  "odom_ready":true,
  "pose_valid":true,
  "estop":false,
  "current_job":null,
  "error":null
}
```

位姿：

```json
{
  "type":"POSE",
  "seq":1,
  "x_mm":0.0,
  "y_mm":0.0,
  "yaw_deg":0.0,
  "vx_world_mm_s":0.0,
  "vy_world_mm_s":0.0,
  "w_rad_s":0.0,
  "timestamp_ms":0.0
}
```

第一阶段只实现以下命令，且不允许产生运动：

- `PING` → `ACK` + `PONG`；
- `STOP` → 停止输出并 ACK；
- `ESTOP` → 电机停止、Pen 抬起并 ACK；
- `CLEAR_ESTOP` → 清除急停并 ACK；
- `RESET_POSE` → 重置里程计并上报 POSE。

第二阶段加入：

- `UPLOAD_BEGIN`；
- 定长二进制接收；
- `UPLOAD_END`；
- `RUN_TRAJECTORY`；
- `ROTATE_REL`。

### 3.4 推荐状态机

```text
BOOT
  -> HELLO_SENT
  -> READY
  -> UPLOADING
  -> READY_WITH_JOB
  -> RUNNING
  -> READY
```

任何阶段收到 `ESTOP` 都进入 `ESTOPPED`。只有收到 `CLEAR_ESTOP` 后才能回到 READY。上传期间禁止 RUN；运行期间禁止覆盖当前 job。

### 3.5 实车接入顺序

1. **空载通信**：只发送 HELLO、STATUS、POSE；UI 必须从 CONNECTING 进入 READY；
2. **安全命令**：车轮架空，验证 STOP、ESTOP、CLEAR_ESTOP、RESET_POSE；
3. **旋转**：架空执行 `+10°`、`-10°`，确认正角度为逆时针；
4. **上传不执行**：上传一个小 TRJ2，检查 size、CRC 和 decoder 日志；
5. **纯 LINE**：轮子接触地面，低速执行 100 mm 直线；
6. **纯 CIRCLE**：验证 CIRCLE 的方向和半径；
7. **Pen 事件**：验证 PEN_UP/PEN_DOWN 顺序和实际 Pen；
8. **地图任务**：最后再执行带障碍绕行的 cheap_map。

## 4. PC 端和 ESP32 端的边界

PC 负责：

- 地图编辑和比例尺；
- Home 到 WORLD 的转换；
- 路径平滑、Polyline 保真、障碍物避让；
- 车体外形安全范围；
- TRJ2 生成、预览和 CRC；
- 上传和运行控制。

ESP32 负责：

- 接收和存储 TRJ2；
- 解码和执行 LINE/CIRCLE/PEN 事件；
- 电机、里程计和 Pen 实时闭环；
- STOP/ESTOP 安全处理；
- 上报真实 POSE/STATUS。

不要把障碍物规划、平滑或比例尺逻辑移到 ESP32。ESP32 只执行 PC 已经编译好的 WORLD 坐标轨迹。

## 5. 必须保持的约定

- WORLD 坐标：`+X` 为小车前方，`+Y` 为小车左方；
- 正 yaw 为逆时针；
- 单位：位置 mm、速度 mm/s、角度 degree；
- TRJ2 header 起点必须等于上传任务开始时的真实里程计位姿，除非明确使用 RESET_POSE；
- Exact Polyline 用于需要保留原始折点的路线；Freehand 会移动折点并圆滑拐角；
- Pen Up 绕障段使用 travel speed，Pen Down 绘图段使用 draw speed；
- 上传成功不等于执行成功，必须由 RUN_TRAJECTORY 明确触发运动；
- 断线、协议错误、CRC 错误和 decoder 错误都必须让 runner 保持安全停止。

## 6. 推荐开发顺序和交付物

### 阶段 A：设置持久化

交付：

- `MapDocument.settings` 或等价的 `MapProjectSettings`；
- UI 保存/加载设置；
- Save 与 Save as 区分；
- round-trip 和旧地图兼容测试。

### 阶段 B：ESP32 N3 空载服务

交付：

- `n3_service.c/h`；
- HELLO、STATUS、POSE、PING；
- PC UI READY 验收；
- 不驱动电机的串口协议测试。

状态：已完成。已在 ESP32 上验收 READY、HELLO、PING、STATUS、POSE 和安全命令。

### 阶段 C：上传服务

交付：

- LittleFS/SPIFFS 初始化；
- UPLOAD_BEGIN/END；
- 长度和 CRC 校验；
- TRJ2 decoder 校验；
- PC 上传成功报告。

状态：已完成代码和离线编译；尚待有板时做一次真实 SPIFFS 上传验收。

### 阶段 D：执行服务

交付：

- runner task；
- RUN_TRAJECTORY；
- STATUS/POSE 实时更新；
- STOP/ESTOP/CLEAR_ESTOP；
- RESET_POSE/ROTATE_REL。

状态：D1 架构整理已完成；D2 Runner 尚未接入。当前安全固件仍会拒绝 RUN/ROTATE，不允许误动车。下一步是在此架构上新增独立 Runner Bridge 和异步事件轨迹执行。

### 阶段 E：真实小车验收

交付：

- 直线、旋转、圆弧、Pen、绕障和完整地图任务记录；
- 串口和 Wi-Fi 两种连接方式；
- 实车参数和最终 `.vmap.json` 配置归档。

## 7. 常用检查命令

PC 工程目录下使用项目解释器：

```powershell
.\.conda\python.exe -s -m pytest -q --tb=short --basetemp .pytest_tmp_handoff -p no:cacheprovider
```

重新生成 cheap_map 当前 TRJ2 验收文件：

```powershell
.\.conda\python.exe -s examples/generate_cheap_map_traj_review.py
```

检查结果目录：

```text
test_results/virtual_map_demo/cheap_map_traj_review/
```

启动 PC UI：

```powershell
.\.conda\python.exe -s -m pc_trajectory.navigation_ui
```

当前刷入前应使用 `build` 目录中的完整产物；刷入后先验收 READY 和上传，不要把当前安全固件当作运动固件。下一阶段才把底盘、里程计、Pen 和 trajectory runner 接到独立的 runner task，并保留完整错误处理。

### 1.8 2026-09-10：M6 motion-circle-pen 实车准备完成

M6 已完成代码和固件构建，正式档位为：

- `N3_ENABLE_HARDWARE=1`；
- `N3_ENABLE_MOTION=1`；
- `N3_ENABLE_CIRCLE=1`；
- `N3_ENABLE_PEN=1`；
- `N3_ENABLE_ROTATE_REL=0`；
- `N3_ENABLE_SIMULATION=0`。

HELLO 应报告 `build_profile=motion-circle-pen`、`execution_mode=HARDWARE_DRAW`。
预检现在接受 LINE、CIRCLE、PEN_UP、PEN_DOWN、WAIT；运动记录仍要求速度
`30..250 mm/s`，圆半径必须大于 `30 mm`（小于 `100 mm` 仅作质量警告），CUBIC_BEZIER 仍拒绝。事件记录不再被
误判为速度为零的运动记录。

实体 Pen 的当前暂定接线配置为 GPIO45、LEDC timer 3、channel 7；UP=1540 us、
DOWN=1100 us，升笔 250 ms、落笔 300 ms、稳定 120 ms，动作超时 2 s。GPIO45、
舵机外部 5 V 供电和共地必须在刷写前由实物再次确认。STOP/ESTOP/错误路径会先
发出急抬笔并等待到 UP 或超时，再进入终态；STATUS 增加 `pen_state`、`pen_target`、
`pen_busy`、`pen_settling`、脉宽和 `pen_error` 字段。

PC 侧新增：

- `examples/generate_n3_pen_fixtures.py`；
- `examples/accept_n3_pen_motion.py`；
- `D:\esp-projects\test-motor\spiffs\n3_pen\n3_pen_only.traj`；
- `n3_pen_line.traj`、`n3_pen_mixed.traj`、`n3_pen_hold_down.traj`；
- `n3_pen_cubic_reject.traj`。

本次构建产物：

```text
D:\esp-projects\test-motor\build-circle\test-motor.bin
D:\esp-projects\test-motor\build-circle\storage.bin
D:\esp-projects\test-motor\build-circle\bootloader\bootloader.bin
```

刷写后使用 `examples/accept_n3_pen_motion.py` 做一次完整验收。脚本会依次验证
READY、Pen 状态转换、PEN LINE、PEN LINE+CIRCLE+LINE、STOP/ESTOP 抬笔、CLEAR_ESTOP
和 CUBIC 拒绝。未完成实车验收前，不要把 GPIO45 脉宽当作最终机械标定值，也不要
执行正式地图任务。

### 1.9 2026-09-10：M7 UI 实车任务预检与起点绑定

PC UI 的 Real Robot 面板现在会针对精确的 TRJ2 字节执行 M6 预检：确认 READY 后的
`motion-circle-pen` 能力、记录数、总路程、预计时长、LINE/CIRCLE 几何、速度和加速度。
预检结果会在上传前显示，硬错误不会打开上传事务；短于 150 mm 的 LINE 只作为提示。

当前实车绘图档位的共同上限为：**1024 条 TRJ2 记录（包含 PEN/WAIT 记录）、30,000 mm
总运动路程、20 分钟执行窗口**。CIRCLE 支持半径 `(30, 5000] mm`、最大扫角 `360 deg`、
单弧长最大 `3000 mm`；半径低于 `100 mm` 仅产生质量警告。速度可在 Toolpath parameters 面板中分别设置：Travel
speed（Pen-up）和 Draw speed（Pen-down），二者范围均为 `30..250 mm/s`，默认 Travel
speed 为 `100 mm/s`。Acceleration 可设为 `50..1000 mm/s²`，默认 `300 mm/s²`。修改后
点击 Apply and replan，再重新预检和上传。

上传结果与精确 TRJ2 CRC 绑定。地图修改或重新规划会使旧上传失效；运行前必须依次完成
Upload、将车摆到计划起点、`Reset pose to planned start`。UI 使用计划头的起始 pose，而
不是旧的固定 Home 重置值。默认 Pen-up 移动速度为 100 mm/s，已与 M6 固件上限同步。

实车一次任务结束后，下一次任务不应再依赖板载 Reset。若 STATUS 表示硬件 `estop=true`
（例如之前出现 MOTOR_STALL），在 Real Robot 面板点击 **Clear E-Stop / re-arm** 即可重新
启用电机并清理终端 runner 状态；随后重新 Upload、Reset pose、Run。真实 ESTOP 或 watchdog
锁存仍不会被 Run 自动绕过。

### 1.6 2026-09-10：M2 guarded ROTATE_REL 调试档位

已新增独立的 `motion-rotate` 编译档位，用于真实底盘接入的第一步。该档位：

- `N3_ENABLE_HARDWARE=1`、`N3_ENABLE_ROTATE_REL=1`；
- `N3_ENABLE_MOTION=0`、`N3_ENABLE_PEN=0`；
- 只允许 `ROTATE_REL`，角度限制为 `+/-30 deg`，速度限制为 `10..45 deg/s`；
- `motion-rotate` 将底层旋转完成误差收紧为约 `1.5 deg`，避免 `+10 deg` 在默认 `5 deg`
  容差处过早结束；
- 复用已有 `chassis_motion` 闭环控制器和真实 MPU/里程计；
- `STOP`、`ESTOP`、`CLEAR_ESTOP` 已接入旋转控制器；
- 运动期间超过 2.5 秒未收到 PC 协议心跳时自动急停并锁存；
- 轨迹中的 LINE/CIRCLE 仍返回 `MOTION_DISABLED`，Pen 仍关闭；
- 旧的 `main.c` 手工测试入口不再被生产 CMake 编译，生产入口固定为 `n3_app.c`。

对应 PC 验收脚本为 `examples/accept_n3_rotate_motion.py`。目标架构单文件编译检查和
PC 端 RobotLink 定向测试已通过；尚未刷写和实车运动验收。刷写前必须架空车轮并确保
急停可触达。

### 1.7 2026-09-10：M3 motion-line 单段直线闭环已实现

ESP32 已新增独立的 `motion-line` 硬件档位。构建参数为：

- `N3_ENABLE_HARDWARE=1`；
- `N3_ENABLE_MOTION=1`；
- `N3_ENABLE_ROTATE_REL=0`；
- `N3_ENABLE_PEN=0`；
- `N3_ENABLE_SIMULATION=0`。

该档位在 `trajectory_stack_impl.c` 中正式链接 `trajectory_tracker` 和
`trajectory_runner`，由 `n3_runner_bridge` 创建独立的异步 runner task。当前只接受
TRJ2 V2 中恰好一条 LINE：长度 `0..100 mm`（零长度拒绝）、速度 `20..60 mm/s`、
加速度 `0..300 mm/s²`。CIRCLE、CUBIC、Pen、多段轨迹会在电机启动前拒绝。

Runner 使用已由 N3 硬件层初始化的电机、MPU6050 和里程计，不重复初始化，也不会自动
重置轨迹起点；验收脚本应先发送 `RESET_POSE(0,0,0)`。STATUS 增加了
`tracker_phase`、`runner_error`、`tracking_error_mm`、参考点和车体速度指令字段。

PC 侧新增：

- `examples/generate_n3_line_fixtures.py`；
- `examples/accept_n3_line_motion.py`；
- `D:\esp-projects\test-motor\spiffs\n3_motion_line.traj`；
- `n3_motion_long.traj`、`n3_motion_circle.traj`、`n3_motion_multi.traj`。

本地 ESP-IDF motion-line 镜像已完成编译链接，产物为：

```text
D:\esp-projects\test-motor\build-motion\test-motor.bin
D:\esp-projects\test-motor\build-motion\storage.bin
D:\esp-projects\test-motor\build-motion\bootloader\bootloader.bin
```

下一步是刷写该镜像后进行 M3 实车验收。首次只执行正常 100 mm LINE；确认方向和停止
机制后，再运行脚本中的 STOP、ESTOP 和非法轨迹拒绝检查。通过前不要打开 Pen，也不要
把 `motion-line` 镜像用于 CIRCLE 或完整地图任务。
