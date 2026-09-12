# `tm_trajectory`

Trace Motion 轨迹文件和执行逻辑的公共接口组件。

| 接口 | 职责 |
| --- | --- |
| `trajectory_decoder.h` | TRJ1/TRJ2 文件头、记录和约束校验。 |
| `trajectory_executor.h`、`trajectory_runner.h` | 轨迹记录分派、执行节拍与停止处理。 |
| `trajectory_tracker.h` | 轨迹跟踪控制。 |
| `pen_control.h` | 笔机构抽象接口。 |

该组件只导出头文件；实际实现由 `tm_n3/trajectory_stack_impl.c` 在轨迹绘图工程中编译一次。使用方应通过 N3 服务或明确的实现单元调用，不能在多个源文件重复定义实现宏。
