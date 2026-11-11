# 本阶段验收

日期：2026-11-11。基线：`里程计实机实验` / `e807519049f3f11f957d8983f11838d7183e0998`。交付分支：`codex/groundstation-multimode`。

## 已完成的自动化检查

在 Windows、CPython 3.12.16、锁定项目依赖下运行 **60 项测试，60 项通过，0 跳过**。C 编解码使用本机 MinGW GCC 13.1.0；视频检查使用隔离的官方 GStreamer wheels（CLI / libraries 1.28.8，plugins 1.28.7）。见 [测试结果](validation-result.json)。本结果不包含物理机器人或真实手柄的测试。

| 范围 | 检查结果 |
| --- | --- |
| 原轨迹回归 | 原有 11 项全部通过，未修改控制算法 |
| 原辅助模块 | AprilTag 检测、插值、旧格式回归和数据集测试通过 |
| 数据兼容 | 现有 86 个轨迹 CSV 均能由原格式加载器读取 |
| 手动输入 | 原轴 / 按钮方向、默认比例、死区、按下沿、重连不重放通过 |
| 轨迹 | 固定启动原点、旋转、±π 跨越、速度映射、限幅 / 斜率、到点与完成通过 |
| 模式控制 | 先停止旧输出、确认后激活、接管未回中时保持停止、拒绝与确认超时通过 |
| 协议 | 10 类消息的 Python/C 共同向量、ID、单位、完整帧 CRC、损坏帧拒绝通过 |
| 时效与会话 | 旧会话、旧代次、乱序、重复请求、源时间过期和剩余有效期通过 |
| 实验记录 | Tag 缺失保留 NX 行、严格插值导出、元数据、MP4 封装收尾通过 |
| 故障隔离 | 相机初始化失败、写盘失败时控制线程继续工作 |
| Qt 界面 | 无手柄 / 无相机 / 无实机启动，模拟操作、按键、页面状态及关闭资源通过 |
| 视频 | 真实 H.264/RTP PT96 发送、解码嵌入、断流提示、恢复、进程退出通过 |

新增模块使用 Ruff 0.17.4 完成格式化和 F 类静态检查；生成代码不手工格式化。

## 模拟闭环

测试通过真实回环 UDP 和生成的 MAVLink 编解码完成：

`能力握手 → 时间同步 → 手动模式确认 → 启用电机 → 前向运动 → 释放输入 → 相对轨迹追踪 → 完成归零 → 停止 → 手动恢复 → 停用电机`

定位无效、里程计重置、350 ms 延迟、全丢包、恢复连接均执行了停止 / 不自动恢复检查。单元测试另外覆盖重复重置请求只执行一次、乱序启用请求不能覆盖停用、旧控制代次不能运动。GStreamer 视频使用 `videotestsrc → x264enc → RTP → UDP → avdec_h264 → RGB pipe → Qt`；展示截图是实际 Qt 渲染。

模拟器为简化平面运动与定深模型，不描述真实 ROV 水动力、推力分配、饱和死区或实际惯性里程计精度。模拟闭环通过不能替代水下实机验证。

## 重现检查

```powershell
uv sync --locked
$env:CC = 'C:\path\to\gcc.exe'
$env:GST_LAUNCH = 'C:\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe'
$env:QT_QPA_PLATFORM = 'offscreen'
uv run python -m unittest discover -v
```

GStreamer 安装目录需正确配置 DLL 与插件搜索路径。缺少 `CC` 或 GStreamer 时对应检查会跳过，输出会明确标识。测试采用临时随机 UDP 端口，不访问配置的实机 IP。

手动故障联调：

```powershell
uv run python -m groundstation.simulator --delay-ms 350
uv run python -m groundstation.simulator --drop 0.2 --duplicate 0.5
uv run python -m groundstation.simulator --invalid-odometry
uv run python -m groundstation.simulator --reject-switch
uv run python -m groundstation.simulator --reset-after 10
```

以上命令分别运行，每次启动一个模拟器；PC 另开终端 `uv run python -m groundstation --nx 127.0.0.1` 并点击连接。内置模拟器也提供对应的故障注入控件。

## 基线数据修正

原 v2 测试把目录匹配文件数固定为 24，但基线已包含额外实验文件。改为验证 manifest 中的 24 条，再单独检查全部 86 个 CSV 的加载兼容性。manifest 的 013 / 014 行仍引用不存在的 crescent 文件，现已指向基线实际存在的 cross 文件，并按文件内容更新范围、点数和时长。没有修改任何轨迹点数据。

## 后续实机阶段

- NX / 嵌入式实现本协议、目标状态动作、确认去重、代次与独立 200 ms 看门狗；接入实际惯性里程计、定深控制和电机输出。
- 实际手柄逐轴 / 逐键核对设备驱动、触发器初值、插拔和再连接；自动化只验证映射与故障状态机。
- 连接实际相机与 NX 视频，核对分辨率、真实传输延迟、断流、录像帧率和驱动关闭行为。
- 水下验证坐标系、yaw 方向、深度零点、控制增益、斜率限制与速度到油门的标定关系。
- 验证实际网络抖动和时间同步误差，最终控制超时由下位机独立执行。

本阶段未修改 NX / 惯性里程计工程和嵌入式工程，未驱动任何物理推进器。
