# PC 端轨迹生成软件交接说明：TRJ2 协议设计与 Phase B 开发路线

> **文档状态：设计冻结 / 开发交接稿**\
> **日期：2026-09-02**\
> **适用范围：PC trajectory compiler + ESP32 trajectory execution
> system**
>
> 本文总结当前已经确认的系统职责、已验证的 TRJ1 V1
> 基线、拟作为下一阶段开发目标的 TRJ2 V2.0 Core 协议，以及 Phase B 的 PC
> 端开发架构。
>
> **重要状态说明：**
>
> -   **TRJ1 V1：已实现、已通过 PC 测试及 ESP32 Decoder
>     交叉验证，作为稳定兼容基线冻结。**
> -   **TRJ2 V2.0 Core：本文所述为下一版协议的冻结设计，尚未表示 ESP32
>     已经实现全部 V2 类型。**
> -   **Phase B：将在本文架构基础上继续开发。**
> -   当前 ESP32 原生可执行 Motion 仍以 **LINE / CIRCLE** 为基线。
> -   **CUBIC_BEZIER** 在 V2 中预留正式类型和 payload，但第一阶段不要求
>     ESP32 原生执行。
> -   **PEN_UP / PEN_DOWN / WAIT** 属于 V2 Event 体系，需要后续同步升级
>     firmware。

------------------------------------------------------------------------

## 1. 项目最终目标

系统最终要实现：

``` text
图片 / CAD / SVG / 文字 / 其他二维图形
                ↓
        PC 端理解二维内容
                ↓
        路径 / 笔画提取
                ↓
        几何简化与拟合
                ↓
        Stroke / Travel 规划
                ↓
        合理的 Motion + Event
                ↓
             .traj 文件
                ↓
              ESP32
                ↓
      连续轨迹执行与闭环跟踪
                ↓
       三轮全向底盘真实运动
                ↓
       Pen / 推杆机构完成绘图
```

系统更接近：

``` text
CNC / XY Plotter / Drawing Robot
```

而不是传统的"给机器人一个目标点，然后导航过去"。

PC 是 **trajectory compiler / toolpath planner**。

ESP32 是 **trajectory execution controller**。

------------------------------------------------------------------------

# 2. PC 与 ESP32 的职责边界

## 2.1 PC 端负责什么

PC 端负责回答：

> **"要画什么，以及应该把它编译成怎样的可执行二维轨迹？"**

主要职责包括：

1.  读取 Bitmap / CAD / SVG / Text 等二维输入。
2.  提取轮廓、中心线、笔画或其他二维几何。
3.  建立统一的 Drawing / Stroke / Geometry 内部模型。
4.  完成 pixel/CAD coordinates → trajectory WORLD mm 的坐标变换。
5.  对路径进行：
    -   去噪；
    -   简化；
    -   曲线拟合；
    -   LINE / ARC / Bézier 等几何识别；
    -   segment 数量优化；
    -   尖角与切线连续性优化。
6.  规划多个 Stroke 的绘制顺序。
7.  规划 Pen Up / Pen Down 与非绘图 Travel。
8.  分配 Motion 的 speed / acceleration 等轨迹参数。
9.  做几何合法性和轨迹质量检查。
10. 提供 Preview / Diagnostics。
11. 最终把内部 Toolpath 编译成 `.traj`。

PC **不应该生成每 10 ms 一个离散控制点的海量轨迹**。

------------------------------------------------------------------------

## 2.2 ESP32 端负责什么

ESP32 端负责：

``` text
.traj
  ↓
trajectory_decoder
  ↓
trajectory_executor
  ↓
trajectory_tracker
  ↓
motor_control
  ↓
wheel control / PWM
  ↓
motors
```

ESP32 负责：

-   `.traj` 二进制解析；
-   Motion segment 几何执行；
-   基于路径弧长的运动推进；
-   acceleration / braking；
-   segment junction lookahead；
-   产生连续 WORLD-frame trajectory reference；
-   odometry 闭环跟踪；
-   WORLD → BODY velocity；
-   chassis motion；
-   motor velocity control；
-   未来执行 PEN / WAIT 等 Event。

PC 不负责：

-   实时 odometry feedback；
-   10 ms 控制循环；
-   BODY `vx / vy / w`；
-   三轮逆运动学；
-   wheel PID / FF；
-   PWM；
-   电机实时控制。

------------------------------------------------------------------------

# 3. 系统最重要的架构原则

必须长期保持：

``` text
PC Internal Representation
        ≠
.traj Binary Representation
```

也就是说，PC 可以理解比 ESP32 协议更丰富的几何：

``` text
Drawing
 └── Stroke
      ├── Line
      ├── Arc
      ├── CubicBezier
      └── future geometry
```

然后通过：

``` text
Toolpath Compiler
```

降低（lowering）成 ESP32 当前能够执行的语言：

``` text
Motion
 ├── LINE
 └── CIRCLE

Event
 ├── PEN_UP
 ├── PEN_DOWN
 └── WAIT
```

未来即使增加 SVG、字体、Spline、NURBS，也不应该要求 ESP32 直接理解所有
CAD primitive。

------------------------------------------------------------------------

# 4. TRJ1 V1：永久冻结的稳定基线

TRJ1 V1 已经通过：

``` text
PC unit tests
        ↓
Golden binary test
        ↓
writer → reader round-trip
        ↓
ESP32 trajectory_decoder cross-check
```

因此 V1 不再因为 Phase B 或 GUI 等上层需求随意修改。

## 4.1 V1 Header

固定：

``` text
magic       = "TRJ1"
version     = 1
header_size = 32
record_size = 44
```

Python：

``` python
HEADER_FMT = "<4sHHIHHfffI"
```

Header：

    Offset   Size Type        Field
  -------- ------ ----------- --------------------
         0      4 char\[4\]   magic = `"TRJ1"`
         4      2 uint16      version = `1`
         6      2 uint16      header_size = `32`
         8      4 uint32      segment_count
        12      2 uint16      record_size = `44`
        14      2 uint16      flags = `0`
        16      4 float32     start_x_mm
        20      4 float32     start_y_mm
        24      4 float32     start_yaw_deg
        28      4 uint32      reserved = `0`

------------------------------------------------------------------------

## 4.2 V1 Record

Python：

``` python
RECORD_FMT = "<BBHff8f"
```

固定 44 bytes：

    Offset   Size Type           Field
  -------- ------ -------------- --------------------
         0      1 uint8          type
         1      1 uint8          flags
         2      2 uint16         reserved
         4      4 float32        speed_mm_s
         8      4 float32        acceleration_mm_s2
        12     32 float32\[8\]   data\[8\]

V1 正式 Motion：

``` text
0 = NONE
1 = LINE
2 = CIRCLE
```

------------------------------------------------------------------------

## 4.3 V1 Golden Test

已验证轨迹：

``` text
Header:
start = (0, 0, 0°)

LINE
(0,0) → (500,0)
speed = 300

CIRCLE
center = (500,250)
radius = 250
start_angle = -90°
sweep = +180°
speed = 220

LINE
(500,500) → (0,500)
speed = 300
```

文件长度：

``` text
32 + 3 × 44 = 164 bytes
```

这份 Golden binary 应永久保留作为协议 regression fixture。

------------------------------------------------------------------------

# 5. 为什么需要 TRJ2

TRJ1 只描述：

``` text
LINE
CIRCLE
```

这对于底盘运动已经足够完成第一阶段验证，但真正的绘图系统还需要表达：

``` text
PEN_UP
PEN_DOWN
WAIT
```

而且 PC 内部未来还需要理解：

``` text
CubicBezier
Stroke
Travel
Drawing
```

因此下一版本必须从"纯 Motion segment 文件"升级成：

``` text
Motion + Event execution stream
```

这就是 TRJ2。

------------------------------------------------------------------------

# 6. TRJ2 V2.0 Core：总体设计

建议正式冻结：

``` text
Header
+
Record 0
+
Record 1
+
...
```

继续保持：

``` text
Header = 32 bytes
Record = 44 bytes
little-endian
IEEE-754 float32 little-endian
```

这样最大程度继承 V1 已经验证的实现。

------------------------------------------------------------------------

# 7. TRJ2 Header

V2 Header：

    Offset   Size Type        Field
  -------- ------ ----------- --------------------
         0      4 char\[4\]   magic = `"TRJ2"`
         4      2 uint16      version = `2`
         6      2 uint16      header_size = `32`
         8      4 uint32      record_count
        12      2 uint16      record_size = `44`
        14      2 uint16      flags = `0`
        16      4 float32     start_x_mm
        20      4 float32     start_y_mm
        24      4 float32     start_yaw_deg
        28      4 uint32      reserved = `0`

Python 仍然：

``` python
HEADER_FMT = "<4sHHIHHfffI"
```

## 7.1 为什么改叫 record_count

V1 的内容全部是 Motion，因此叫：

``` text
segment_count
```

没有问题。

V2 中可能出现：

``` text
PEN_DOWN
LINE
CIRCLE
PEN_UP
WAIT
```

因此文件里不再全部是 geometry segment。

V2 应把 offset 8 的语义正式定义为：

``` text
record_count
```

------------------------------------------------------------------------

# 8. TRJ2 Record

仍然固定：

``` text
44 bytes
```

布局：

    Offset   Size Type           Field
  -------- ------ -------------- --------------------
         0      1 uint8          type
         1      1 uint8          flags
         2      2 uint16         reserved
         4      4 float32        speed_mm_s
         8      4 float32        acceleration_mm_s2
        12     32 float32\[8\]   data\[8\]

Python：

``` python
RECORD_FMT = "<BBHff8f"
```

------------------------------------------------------------------------

# 9. V2 Record Type 空间

为了避免 Motion 和 Event 混成一个随意增长的 enum，建议冻结类型范围：

  Type range    Meaning
  ------------- ---------------------------
  `0x00`        NONE / invalid
  `0x01–0x1F`   Motion Geometry
  `0x20–0x3F`   Tool / Event
  `0x40–0x7F`   Future control / metadata
  `0x80–0xFF`   Reserved

Core types：

``` python
REC_NONE          = 0x00

REC_LINE          = 0x01
REC_CIRCLE        = 0x02
REC_CUBIC_BEZIER  = 0x03

REC_PEN_UP        = 0x20
REC_PEN_DOWN      = 0x21
REC_WAIT          = 0x22
```

------------------------------------------------------------------------

# 10. Motion Record

Motion 会改变 logical XY position。

Motion 的公共规则：

``` text
speed_mm_s > 0
acceleration_mm_s2 >= 0
```

其中：

``` text
acceleration == 0
```

表示文件未指定 acceleration constraint，由 Executor 使用自己的默认值。

------------------------------------------------------------------------

# 11. LINE

``` text
type = 0x01
```

Payload：

``` text
data[0] = end_x_mm
data[1] = end_y_mm
data[2..7] = 0
```

LINE 不保存 start。

其起点为：

``` text
current logical XY
```

第一个 Motion 的 logical start 来自：

``` text
header.start_x_mm
header.start_y_mm
```

后续 Motion 的 start 来自前一个 Motion 的终点。

------------------------------------------------------------------------

# 12. CIRCLE / ARC

``` text
type = 0x02
```

Payload：

``` text
data[0] = center_x_mm
data[1] = center_y_mm
data[2] = radius_mm
data[3] = start_angle_deg
data[4] = sweep_deg
data[5..7] = 0
```

定义：

``` text
x(theta) = center_x + radius * cos(theta)
y(theta) = center_y + radius * sin(theta)

theta_start = start_angle_deg
theta_end   = start_angle_deg + sweep_deg
```

方向：

``` text
sweep > 0 → CCW
sweep < 0 → CW
```

CIRCLE 的隐式几何起点必须与当前 logical XY 连续。

------------------------------------------------------------------------

# 13. CUBIC_BEZIER

``` text
type = 0x03
```

## 13.1 Payload

起点：

``` text
P0 = current logical XY
```

Record：

``` text
data[0] = control1_x_mm
data[1] = control1_y_mm

data[2] = control2_x_mm
data[3] = control2_y_mm

data[4] = end_x_mm
data[5] = end_y_mm

data[6] = 0
data[7] = 0
```

因此：

``` text
P0 = implicit current point
P1 = control point 1
P2 = control point 2
P3 = endpoint
```

数学定义：

``` text
P(u) =
(1-u)^3 P0
+ 3(1-u)^2 u P1
+ 3(1-u) u^2 P2
+ u^3 P3

0 <= u <= 1
```

## 13.2 当前支持状态

必须明确区分：

``` text
PC Internal Geometry:
SUPPORTED

TRJ2 Type / Payload:
DEFINED

Default Toolpath Compiler:
暂不要求输出 native Bézier

ESP32 baseline:
暂不要求原生执行
```

第一阶段默认：

``` text
CubicBezier
      ↓
curve fitting / approximation
      ↓
LINE + CIRCLE
      ↓
TRJ2
```

未来如果证明 native Bézier 对字体、SVG 或复杂曲线有明显价值，再实现
ESP32：

``` text
arc length s(u)
inverse u(s)
position
unit tangent
curvature
```

然后开启 native `0x03` execution。

------------------------------------------------------------------------

# 14. 为什么只定义 Cubic Bézier

当前不建议同时加入：

``` text
QuadraticBezier
Spline
B-Spline
NURBS
```

原因：

-   Quadratic Bézier 可以精确转换成 Cubic Bézier；
-   SVG 等系统大量使用 Cubic Bézier；
-   复杂 spline 可以在 PC 端拆成多个 Cubic Bézier；
-   ESP32 execution language 应保持小而稳定。

因此 PC IR 可以不断扩展，但 `.traj` core primitive 不需要无限增加。

------------------------------------------------------------------------

# 15. Event Record

Event 与 Motion 的根本区别：

> **Event 不改变 logical XY position。**

Core Event：

``` text
PEN_UP
PEN_DOWN
WAIT
```

Event 公共规则：

``` text
speed_mm_s = 0
acceleration_mm_s2 = 0
```

Event 是 **blocking zero-speed barrier**。

------------------------------------------------------------------------

# 16. PEN_UP

``` text
type = 0x20

flags = 0
reserved = 0

speed = 0
acceleration = 0

data[0..7] = 0
```

语义：

``` text
保持当前位置
↓
Motion speed = 0
↓
执行 pen_up()
↓
等待机构动作完成
↓
继续下一 record
```

`.traj` 不描述：

``` text
servo angle
PWM
推杆行程
GPIO
actuator timing
```

这些属于：

``` text
pen_control
```

硬件配置。

------------------------------------------------------------------------

# 17. PEN_DOWN

``` text
type = 0x21

flags = 0
reserved = 0

speed = 0
acceleration = 0

data[0..7] = 0
```

语义：

``` text
保持当前位置
↓
Motion speed = 0
↓
执行 pen_down()
↓
等待机构动作完成
↓
继续下一 record
```

------------------------------------------------------------------------

# 18. WAIT

``` text
type = 0x22

speed = 0
acceleration = 0
```

Payload：

``` text
data[0] = duration_s
data[1..7] = 0
```

要求：

``` text
duration_s > 0
finite
```

语义：

``` text
XY hold
yaw hold
pen state unchanged
wait duration_s
```

------------------------------------------------------------------------

# 19. Pen 初始状态

TRJ2 正式规定：

``` text
Logical initial pen state = UP
```

这样：

``` text
PEN_DOWN
LINE
```

表示从 trajectory start 立即开始绘图。

而：

``` text
LINE
PEN_DOWN
```

表示先以 pen-up 状态移动到绘图起点。

建议 firmware 安全策略：

``` text
trajectory start → PEN_UP
normal end       → PEN_UP
cancel           → PEN_UP
error            → PEN_UP
```

这属于 Runner / pen_control 的执行安全策略。

------------------------------------------------------------------------

# 20. 不定义 DRAW_LINE / TRAVEL_LINE

不需要：

``` text
DRAW_LINE
TRAVEL_LINE
```

是否绘图由当前 pen state 决定。

例如：

``` text
PEN_UP
LINE speed=600
```

自然表示快速空行程。

而：

``` text
PEN_DOWN
LINE speed=250
```

自然表示绘图 Motion。

这能避免重复几何类型。

------------------------------------------------------------------------

# 21. Event 与 logical position

例如：

``` text
LINE → (100,100)

PEN_UP
WAIT
PEN_DOWN

LINE → (200,100)
```

执行：

``` text
PEN_UP
WAIT
PEN_DOWN
```

期间：

``` text
logical XY = (100,100)
```

始终不变。

因此第二个 LINE 的隐式 start 仍然：

``` text
(100,100)
```

------------------------------------------------------------------------

# 22. Event 是速度屏障

Motion → Motion：

``` text
允许 Executor 根据 position/tangent continuity
决定是否保持非零 junction speed
```

Motion → Event：

``` text
ALWAYS exit_speed = 0
```

Event → Motion：

``` text
ALWAYS start_speed = 0
```

例如：

``` text
LINE
PEN_UP
LINE
```

即使两条 LINE 完全共线，也必须：

``` text
decelerate
→ stop
→ PEN_UP
→ accelerate
```

------------------------------------------------------------------------

# 23. Motion 几何连续性

没有 Event 隔开的 Motion 应满足：

``` text
position continuity
```

并尽量满足：

``` text
tangent continuity
```

当前 firmware V1 Executor 的重要基线：

``` text
firmware position continuity tolerance = 2 mm
firmware tangent tolerance             = 5°
```

但 PC generator 不应依赖 2 mm 的 firmware 容错。

建议 PC 使用远严格于 firmware 的生成容差，例如：

``` text
PC continuity tolerance ≈ 0.01 mm
```

这里应区分：

``` text
Firmware acceptance tolerance
```

和：

``` text
PC generation quality requirement
```

------------------------------------------------------------------------

# 24. start pose 语义

Header：

``` text
start_x_mm
start_y_mm
start_yaw_deg
```

不是 metadata，而是 execution contract。

其中：

``` text
start_x / start_y
```

定义 trajectory WORLD 的起始 logical XY。

而：

``` text
start_yaw_deg
```

表示 chassis fixed heading target。

它 **不是**：

``` text
first segment tangent
```

三轮全向底盘允许：

``` text
chassis yaw = 0°
```

同时沿 WORLD 任意二维方向运动。

因此 PC 不应根据第一段 trajectory tangent 自动修改 start_yaw。

------------------------------------------------------------------------

# 25. TRJ2 Canonical Writer Rules

PC writer 应只产生一种 canonical TRJ2。

必须：

``` text
magic       = "TRJ2"
version     = 2
header_size = 32
record_size = 44
flags       = 0
reserved    = 0
```

所有未使用：

``` text
flags
reserved
data[]
```

必须写 0。

文件长度必须严格：

``` text
file_size = 32 + record_count * 44
```

不允许：

``` text
trailing bytes
```

------------------------------------------------------------------------

# 26. 两 Stroke 示例

目标：

``` text
Stroke A:
(0,0) → (100,0)

Stroke B:
(200,0) → (300,0)
```

PC Internal Drawing：

``` text
Drawing
├── Stroke A
│   └── Line
└── Stroke B
    └── Line
```

Toolpath Compiler 可以生成：

``` text
Header
start = (0,0,0)

0: PEN_DOWN

1: LINE
   → (100,0)

2: PEN_UP

3: LINE
   → (200,0)
   # travel

4: PEN_DOWN

5: LINE
   → (300,0)

6: PEN_UP
```

因此：

``` text
record_count = 7
```

文件长度：

``` text
32 + 7 × 44
= 340 bytes
```

这说明：

``` text
Drawing / Stroke
```

和：

``` text
TRJ2 Record Stream
```

是两个不同层次的数据结构。

------------------------------------------------------------------------

# 27. PC 端推荐的最终分层

建议逐渐形成：

``` text
pc_trajectory/
│
├── geometry/
│   ├── primitives.py
│   ├── line.py
│   ├── arc.py
│   └── bezier.py
│
├── drawing/
│   ├── drawing.py
│   └── stroke.py
│
├── toolpath/
│   ├── motion.py
│   ├── event.py
│   ├── compiler.py
│   ├── analysis.py
│   └── ordering.py
│
├── export/
│   ├── traj_v1.py
│   └── traj_v2.py
│
├── preview/
│   └── ...
│
└── input/
    ├── raster/
    ├── svg/
    ├── cad/
    └── text/
```

第一阶段不必立即建立所有目录，但架构依赖方向应遵循这一思路。

------------------------------------------------------------------------

# 28. PC Internal Representation

建议内部模型至少包含：

``` text
Drawing
 ├── Stroke
 │    ├── Geometry
 │    ├── Geometry
 │    └── ...
 │
 ├── Stroke
 │    └── ...
 │
 └── ...
```

Geometry：

``` text
Line
Arc
CubicBezier
```

未来可以扩展：

``` text
Polyline
Spline
...
```

但这些内部类型不必一一成为 `.traj` type。

------------------------------------------------------------------------

# 29. Toolpath Compiler 的职责

Toolpath Compiler 是整个 PC 软件最重要的中间层。

输入：

``` text
Drawing / Stroke / Geometry
```

输出：

``` text
ordered Motion + Event stream
```

它负责：

1.  Stroke ordering；
2.  Stroke direction selection；
3.  travel planning；
4.  PEN_UP / PEN_DOWN 插入；
5.  Geometry lowering；
6.  curve simplification / fitting；
7.  LINE / CIRCLE segmentation；
8.  future native Bézier selection；
9.  speed assignment；
10. acceleration assignment；
11. continuity optimization；
12. diagnostics。

------------------------------------------------------------------------

# 30. Phase B 的重新定位

Phase B 不再只是：

``` text
方便人工调用 line_to() / arc()
```

而是开始建设整个 trajectory compiler 的：

> **几何中间层 + Toolpath Backend**

Phase B 应重点建立：

``` text
Geometry Engine
Drawing / Stroke Model
Toolpath Model
Path Analysis
Manual Builder
Preview
TRJ1 Export Integration
TRJ2 Data Model
```

但暂时不进入复杂 OpenCV。

------------------------------------------------------------------------

# 31. Phase B 推荐开发顺序

## B1 --- Geometry Primitives

实现：

``` text
Point2D
Vector2D

Line
Arc
CubicBezier
```

支持：

``` text
start
end
length
point_at()
tangent_at()
bounding box
```

其中 CubicBezier 的精确 arc-length / native ESP32 execution 暂不要求。

------------------------------------------------------------------------

## B2 --- Drawing / Stroke

实现：

``` text
Drawing
Stroke
```

明确区分：

``` text
同一连续笔画
```

与：

``` text
不同笔画
```

这是未来 Pen Planning 的基础。

------------------------------------------------------------------------

## B3 --- Toolpath Model

建立独立于 binary record 的：

``` text
Motion
Event
Toolpath
```

例如：

``` text
Motion(Line)
Motion(Arc)

PenUp
PenDown
Wait
```

------------------------------------------------------------------------

## B4 --- Geometry Analysis

实现：

``` text
segment start/end
segment length
start/end tangent
signed curvature
position continuity
junction angle
tangent continuity
total length
bounding box
short segment warning
sharp junction warning
```

这里区分：

``` text
ERROR
WARNING
INFO
```

合法轨迹不等于优质轨迹。

------------------------------------------------------------------------

## B5 --- Manual Builder

提供人类友好的 API，例如：

``` python
drawing = Drawing()

stroke = drawing.new_stroke()
stroke.line_to(...)
stroke.arc_to(...)
stroke.cubic_to(...)
```

或者底层 Toolpath Builder。

Builder 不直接 `struct.pack()`。

------------------------------------------------------------------------

## B6 --- Preview

Preview 应至少显示：

``` text
Drawing geometry
Stroke boundaries
Motion direction
Travel path
Start / End
Sharp junction
Segment index
WORLD axes
mm units
```

以后可以进一步区分：

``` text
drawing motion
travel motion
pen events
```

Preview 采样点只用于屏幕显示，绝不写成 10 ms trajectory points。

------------------------------------------------------------------------

## B7 --- Export

Phase B 必须保留：

``` text
TRJ1 exporter
```

作为稳定 baseline。

同时建立：

``` text
TRJ2 model / exporter tests
```

在 firmware V2 尚未完成前，可以控制：

``` text
native Bézier export = disabled
event export = development / test mode
```

------------------------------------------------------------------------

# 32. Phase B 与 Phase A 的关系

Phase A 已完成：

``` text
TRJ1 binary protocol backend
```

Phase B 不应破坏 Phase A。

最终：

``` text
             Drawing
                ↓
              Stroke
                ↓
             Geometry
                ↓
        Toolpath Compiler
                ↓
         Motion + Event
           /         \
          /           \
     TRJ1 Export    TRJ2 Export
        ↓              ↓
   LINE/CIRCLE     Motion + Event
```

所有 Phase A regression tests 必须继续 PASS。

------------------------------------------------------------------------

# 33. 后续图片 / CAD 阶段

当 Phase B 稳定后，再开发输入前端：

``` text
Raster Image
    ↓
grayscale / binarization
    ↓
contour / centerline extraction
    ↓
Stroke

SVG / CAD
    ↓
vector parsing
    ↓
Geometry

Text
    ↓
font outline
    ↓
Geometry
```

然后全部进入同一个：

``` text
Drawing
↓
Toolpath Compiler
↓
.traj
```

OpenCV、SVG parser、CAD parser 不允许直接调用 `struct.pack()`。

------------------------------------------------------------------------

# 34. 关于原生 Bézier 的决策原则

当前先不急着让 ESP32 原生执行 Bézier。

后续用真实数据判断：

``` text
一条 CubicBezier
↓
如果通常只需要少量 LINE/CIRCLE
即可达到所需误差
```

则没有必要增加 firmware complexity。

如果字体 / SVG 大量出现：

``` text
1 Bezier
→ 数十甚至上百 LINE/CIRCLE records
```

并明显导致：

``` text
文件膨胀
segment 数量过多
junction 质量下降
执行效率下降
```

再实现 native Bézier。

因此：

> 是否原生支持 Bézier，应由真实 toolpath 数据决定，而不是仅因为 CAD
> 中存在 Bézier。

------------------------------------------------------------------------

# 35. ESP32 V2 后续升级方向

V2 firmware 预计需要增加：

``` text
trajectory_decoder_v2
        ↓
Motion / Event record
        ↓
trajectory_executor
        ├── Motion
        │    ├── LINE
        │    ├── CIRCLE
        │    └── future BEZIER
        │
        └── Event
             ├── PEN_UP
             ├── PEN_DOWN
             └── WAIT
```

Tracker 原则上仍然只面对：

``` text
trajectory_reference_t
```

不应该知道：

``` text
LINE
CIRCLE
BEZIER
PEN
```

Pen event 应由：

``` text
Executor / Runner / Event Layer
```

调用：

``` text
pen_control
```

而不是进入 Tracker。

------------------------------------------------------------------------

# 36. 当前冻结的核心决定

以下作为当前 TRJ2 V2.0 Core 的设计基线：

1.  **TRJ1 V1 永久冻结并保留。**
2.  **下一版使用 `"TRJ2"` / version `2`。**
3.  **Header 继续固定 32 bytes。**
4.  **Record 继续固定 44 bytes。**
5.  **V2 offset 8 的语义改为 `record_count`。**
6.  **Motion 与 Event 正式分离。**
7.  **Motion Core：LINE / CIRCLE。**
8.  **CUBIC_BEZIER 分配正式 ID 和 payload，但 baseline firmware
    可暂不支持 native execution。**
9.  **Event Core：PEN_UP / PEN_DOWN / WAIT。**
10. **Event 不改变 logical XY。**
11. **Event 是 blocking zero-speed barrier。**
12. **初始 logical pen state = UP。**
13. **Pen hardware 参数不进入 `.traj`。**
14. **不定义 DRAW_LINE / TRAVEL_LINE；由 pen state 区分。**
15. **start_yaw 继续表示 chassis fixed heading target，不是 path
    tangent。**
16. **PC Internal Geometry 可以比 `.traj` 更丰富。**
17. **Toolpath Compiler 负责把复杂 geometry lowering 成目标 `.traj`
    能力。**
18. **PC 不生成 10 ms 离散控制点。**
19. **Tracker 继续只消费统一 trajectory reference，不感知 geometry/event
    类型。**
20. **是否让 ESP32 原生支持 Bézier，由后续真实路径数据决定。**

------------------------------------------------------------------------

# 37. 当前项目状态

截至本文：

``` text
Phase A
========

TRJ1 V1 protocol layer
✓ format
✓ validation
✓ writer
✓ reader
✓ Golden binary
✓ PC tests
✓ ESP32 decoder cross-check

STATUS: PASS / FROZEN
```

下一阶段：

``` text
Phase B
========

Geometry Engine
Drawing / Stroke Model
Toolpath Model
Motion / Event Model
Path Analysis
Manual Builder
Preview
TRJ1 integration
TRJ2 model / tests

STATUS: NEXT
```

Phase B 完成后再进入：

``` text
Image / SVG / CAD / Text
        ↓
automatic path extraction
        ↓
curve fitting
        ↓
path ordering
        ↓
pen/travel planning
        ↓
toolpath
        ↓
.traj
```

------------------------------------------------------------------------

# 38. 一句话交接

本项目 PC 端不是"PC 控制小车"，而是在开发一个：

``` text
二维图形
  ↓
Drawing / Stroke / Geometry
  ↓
Toolpath Planning
  ↓
Motion + Event
  ↓
TRJ1 / TRJ2
  ↓
.traj
```

的 **trajectory compiler / toolpath planner**。

ESP32 则负责：

``` text
.traj
↓
decode
↓
execute geometry/events
↓
trajectory reference
↓
closed-loop tracking
↓
motor control
↓
真实运动
```

后续开发必须保持这条职责边界，避免把 CAD/image processing、实时控制和
binary serialization 混在同一层。

------------------------------------------------------------------------

## 39. 下一位开发者首先应该做什么

如果接手时 Phase B 尚未开始，请按以下顺序推进：

``` text
1. 不修改已经通过验证的 TRJ1 V1 backend

2. 建立 PC Internal Geometry：
   Point / Vector / Line / Arc / CubicBezier

3. 建立：
   Drawing / Stroke

4. 建立：
   Toolpath / Motion / Event

5. 实现 geometry analysis：
   endpoint / length / tangent / continuity /
   junction / bounds / diagnostics

6. 实现 manual builder

7. 实现 preview

8. 用新的内部模型重新构建 Phase A Golden trajectory

9. 要求 TRJ1 exporter 输出仍与原 164-byte Golden binary 完全一致

10. 建立 TRJ2 data model 和 binary tests

11. 再同步升级 ESP32 对 PEN_UP / PEN_DOWN / WAIT 的支持

12. 最后进入 Image / SVG / CAD / Text 自动路径生成
```

如果任何开发需要修改：

``` text
TRJ1 binary layout
TRJ2 Core type semantics
Motion/Event 边界
start pose 语义
Pen event 语义
```

应先停止开发并重新确认协议，而不是在某个模块内部自行改变格式。
