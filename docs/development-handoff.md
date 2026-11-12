# NXTelemetryMonitor 开发交接文档

交接日期：2026-11-12。适用对象：继续开发上位机、NX 中间件和 FineSUB 嵌入式通信的开发者。

当前已完成多模式上位机、MAVLink 2 应用协议和模拟 NX，可以在本机验证手动操控、相对轨迹追踪、模式切换、故障停止和实验记录。下一阶段的重点是把同一接口接入真实 NX 和嵌入式执行端，使模式切换与超时保护贯穿整条控制链。

## 1. 交付版本与入口

| 项目 | 内容 |
| --- | --- |
| 仓库 | [ganju2390/NXTelemetryMonitor](https://github.com/ganju2390/NXTelemetryMonitor) |
| 交付分支 | [`codex/groundstation-multimode`](https://github.com/ganju2390/NXTelemetryMonitor/tree/codex/groundstation-multimode) |
| 原始基线 | `里程计实机实验`，`e807519049f3f11f957d8983f11838d7183e0998` |
| 功能交付提交 | `c90e9d3`，交接文档随后单独提交；以该分支最新提交获取完整资料 |
| 当前工作副本 | `D:\FINS\NXTelemetryMonitor` |
| 新程序入口 | `python -m groundstation`；`run_monitor.bat` 和原脚本入口也进入新界面 |
| 后续安排 | [分阶段开发计划](roadmap.md) |

本次功能提交依次为：

| 提交 | 内容 |
| --- | --- |
| `63efe3b` | MAVLink XML、Python/C 编解码、模式仲裁、手柄映射、模拟 NX、协议与控制测试 |
| `2c8fddb` | PySide6 界面、控制与输入线程、GStreamer、AprilTag 相机、CSV/MP4、集成测试 |
| `d75137c` | 操作说明、协议文档、验收结果和界面截图 |
| `c90e9d3` | 标记生成文件，便于 GitHub 审阅并保留生成器原始格式 |

## 2. 已实现与待实现边界

| 功能 | 当前状态 | 交接要点 |
| --- | --- | --- |
| 手柄与键盘 | 上位机已实现 | 保留 V5StreamerNX 映射；输入源互斥；设备失联后需明确恢复 |
| 水平轨迹 | 上位机与模拟闭环已实现 | 轨迹规划和速度到油门映射在 PC；只用 NX 里程计反馈 |
| 定深 | 目标发送与模拟行为已实现 | 默认捕获启动深度；真实垂直控制器尚未接入 |
| 模式切换 | PC 与模拟 NX 已实现 | 先清零旧输出，新代次收到匹配确认才激活；与电机启用分离 |
| MAVLink 2 | 定义、生成代码与共同向量已实现 | 真实 NX / MCU 仍需实现消息语义、确认和有效期 |
| 视频 | GStreamer 接收与 Qt 显示已实现 | 保持 H.264/RTP、UDP 5600、PT 96；真实 NX 视频待联调 |
| AprilTag 与记录 | 独立相机、对照定位、CSV、MP4 已实现 | AprilTag 不参与轨迹反馈；缺 Tag 仍记录 NX 数据 |
| NX 手柄桥接与惯性里程计统合 | 待实现 | 建立统一消息入口、里程计发布和单一串口所有者 |
| FineSUB 消息收发及实际执行 | 待实现 | 绝对目标状态、去重、代次、独立超时保护和定深执行 |
| 实机结果 | 待验证 | 手柄物理插拔、实际相机、实际链路时延与水下轨迹精度 |

本分支的新界面只使用 MAVLink 2，旧 40/52 字节链路不能直接与其互通。旧代码和相关测试保留作迁移参考；[历史说明](legacy_monitor.md)不作为新系统的接口依据。

## 3. 接手后的第一次运行

新机器从交付分支检出，Python 与依赖统一通过 uv 建立：

```powershell
git clone --branch codex/groundstation-multimode --single-branch https://github.com/ganju2390/NXTelemetryMonitor.git
cd NXTelemetryMonitor
uv sync --locked
uv run python -m groundstation --simulate
```

在已有工作副本中先检查 `git status`，再切换交付分支，保留未提交工作。当前开发机也可直接运行：

```powershell
cd D:\FINS\NXTelemetryMonitor
.\run_monitor.bat --simulate
```

操作顺序：选择“键盘备用”→释放运动按键→进入手动模式→启用电机→测试 W/S、A/D、Q/E、R/F→释放按键→选择 CSV→启动轨迹→停止运动→手动恢复→电机停用。界面应始终明确显示 `SIMULATED NX · 模拟运行`。

也可以分开启动两端，方便替换模拟器和注入故障：

```powershell
# 终端 A
uv run python -m groundstation.simulator --bind 127.0.0.1 --pc 127.0.0.1
# 终端 B：随后在界面点击“连接 NX”
uv run python -m groundstation --nx 127.0.0.1
```

默认 PC 接收 54321、NX 接收 54322。GStreamer 属于外部运行库，无视频环境也可测试控制与模拟。详细安装、视频测试源与相机配置见[操作说明](groundstation.md)。

## 4. 代码阅读顺序与职责

建议先读 `models → modes → controller → runtime → simulator`，理解控制流后再读界面和媒体模块。

| 文件 | 责任与修改入口 |
| --- | --- |
| [`protocol/finsrov.xml`](../protocol/finsrov.xml) | 消息 ID、字段类型、单位、枚举的唯一来源 |
| [`groundstation/models.py`](../groundstation/models.py) | 模式、动作、输入、里程计、统一机体指令和配置 |
| [`groundstation/inputs.py`](../groundstation/inputs.py) | 原手柄映射、死区、热插拔、按下沿检测 |
| [`groundstation/modes.py`](../groundstation/modes.py) | `enter/update/exit` 接口、手动与轨迹模式、固定启动原点 |
| [`trajectory_tracking.py`](../trajectory_tracking.py) | 既有逐点规划、误差修正、坐标变换、限幅和斜率限制 |
| [`groundstation/controller.py`](../groundstation/controller.py) | 唯一命令发送仲裁器、会话、代次、确认、故障停止 |
| [`groundstation/runtime.py`](../groundstation/runtime.py) | 50 Hz 控制线程、输入线程、队列交换和状态快照 |
| [`groundstation/wire.py`](../groundstation/wire.py) | UDP 接收/发送与生成的 MAVLink 编解码适配 |
| [`groundstation/simulator.py`](../groundstation/simulator.py) | NX 接收规则参考、简化运动/定深模型、故障注入 |
| [`groundstation/video.py`](../groundstation/video.py) | GStreamer 子进程、RGB 管道、最新帧队列与退出管理 |
| [`groundstation/camera.py`](../groundstation/camera.py) | 独立本地相机、AprilTag 对照与 MP4 |
| [`groundstation/recording.py`](../groundstation/recording.py) | 异步会话记录、原九项预测量、Tag 插值、质量标志 |
| [`groundstation/app.py`](../groundstation/app.py) | Qt 界面、操作者动作、输入预览、曲线与媒体展示 |

控制算法不访问 Qt、socket 或相机；各模式返回统一 `Setpoint`，由仲裁器发送。控制、网络、输入、相机和写盘各有独立工作线程，GStreamer 解码另用进程。跨线程数据通过有界队列传递，视频积压只保留最新帧。

新增控制模态时扩展模式实现、注册入口和 XML 能力定义，维持统一输出契约；不要另建绕过仲裁器的运动发送路径。

## 5. 后续开发必须保持的接口约定

- **坐标系：** 机体前、左、上为正；世界 XY 为 NX 局部里程计坐标，yaw 逆时针为正；深度独立向下为正。单位 m、m/s、rad、rad/s。
- **控制量：** PC 输出归一化前后、左右、偏航及垂直指令，不输出八个推进器的 PWM。手柄轴只在 PC 映射一次，NX 不应再次反向或缩放。
- **轨迹：** 首点局部 XY 为零，启动确认时固定 NX 位置与航向为原点。位置误差 ≤0.15 m 且航向误差 ≤10°才推进。完成后水平/偏航归零并保持目标深度；点击停止撤销定深输出。
- **时序：** 控制 50 Hz、心跳 1 Hz；运动有效期最多 200 ms。定位同时校验源时间和接收时间；迟到报文不能因转发而重新获得完整 200 ms。
- **动作：** 启停与灯光采用绝对目标状态；300 ms 未确认重发同一请求，最多重试三次。去重后仅重复 ACK，不重复执行动作。
- **代次与恢复：** 会话、代次和序号共同约束指令；故障或重连后需要明确恢复。里程计重置改变 `reset_id` 并终止当前轨迹。
- **执行状态：** ACK 代表执行端已接受并应用的状态；不能仅因为 NX 收到 UDP 就宣称 MCU 电机已启用。NX 尚未接入某项能力时不应声明支持。

字段、ID、拒绝原因及 CRC 验证方法见[协议文档](protocol.md)；上述规则应在真实 NX 和 MCU 上继续测试。

## 6. 验收证据与重现方法

2026-11-11 功能验收记录为 **60 项通过、0 失败、0 错误、0 跳过**，包含原有 11 项轨迹回归、86 个轨迹 CSV 加载、Python/C 共同向量、真实 UDP 模拟闭环、Qt 操作、GStreamer RTP 断流恢复、CSV 与 MP4 收尾。见[验收说明](validation.md)和[机器可读结果](validation-result.json)。本次交接文档更新不改变运行代码。

```powershell
uv sync --locked
# 完整验收另需 C11 编译器与 GStreamer
$env:CC = 'C:\path\to\gcc.exe'
$env:GST_LAUNCH = 'C:\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe'
$env:QT_QPA_PLATFORM = 'offscreen'
uv run python -m unittest discover -v
```

缺少编译器或 GStreamer 的检查会明确跳过，不能据此声称全项验收通过。测试使用回环随机端口，不向实机发送动作。Qt 的 offscreen 设置用于自动化；日常操作可在新终端正常启动，或移除该环境变量。

协议发生变动时先改 XML，再运行 `uv run python tools/generate_protocol.py`，同步 Python/C 生成物和测试向量。当前测试使用 unittest；按既有结构添加测试即可。

CSV 会话包含 `telemetry.csv`、`aligned.csv`、`session.json`、选中时的 `trajectory.csv` 与 `result.json`。记录中的 PC 输出是接收采样时已知的最新命令；严格插值仅针对 Tag 与 NX 源时间，不代表命令与源采样时刻严格对齐。

## 7. 接入真实链路前的资料清点

以下为本地已找到的入口，现有 NX 与 MCU 的运行版本仍应从设备核对：

| 资料 | 已核对内容 | 下一步 |
| --- | --- | --- |
| `D:\FINS\tools\NXStreamer\nx_streamer.py` | 旧 40/52 字节桥接；串口默认 `/dev/ttyTHS1`、115200；目录目前不是独立 Git 仓库 | 核对 NX 实际部署文件与服务，纳入可追溯版本管理 |
| 开源平台仓库 `software/jetson/V5StreamerNX.py` | 旧手柄消息与轴/按钮映射参考 | 迁移时复用含义，新链路统一收归一化机体指令 |
| `D:\FINS\FineSUB` | 存在 `SUB`、`惯性里程计实验` 分支 | 对比两分支通信、推进器、定深与状态回传；选定基线并新建集成分支 |
| NX 惯性里程计运行入口 | 本轮交接尚未确定唯一部署入口、模型与配置版本 | 列入下一阶段第一项清点，核对输出坐标系、单位、源时刻与重置规则 |

开发接入前需要确认的项目参数集中列在[后续安排](roadmap.md#待确认资料)，无需为本次云端交付补齐这些资料。

## 8. 已知限制与接手注意事项

真实 NX / MCU 尚未执行本协议；生成 C 头文件并不等于完成嵌入式接入。模拟模型没有真实水动力，当前比例参数只能作为联调起点，实机速度到油门的映射和定深参数需要标定。

PC 的 50 Hz 循环不是硬实时任务，真实链路应在 NX、MCU 各自实现超时停止。MAVLink 当前未启用签名，控制权按配置对端与会话处理，多控制站仲裁不在本阶段范围。

真实手柄驱动、触发器初值、物理插拔以及真实相机驱动退出仍需设备验证。当前测试已经验证映射和失联状态机，但不能替代这些设备行为。

现有 v2 manifest 中 013/014 的陈旧文件名已改为实际 cross 文件，轨迹点数据未更改。`nx_telemetry_monitor.py` 仍保留历史类供回归使用，今后的功能开发放入 `groundstation/`。
