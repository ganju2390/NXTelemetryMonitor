# FinsROV MAVLink 2 · 应用协议 v1

定义源：[finsrov.xml](../protocol/finsrov.xml)。由锁定版本 pymavlink 生成 [Python](../groundstation/generated/finsrov.py) 和 [C](../protocol/generated/c/finsrov/mavlink.h)。生成器使用随依赖固定的 `minimal.xml` 提供 HEARTBEAT；TIMESYNC 使用标准基字段及 CRC。遵循 [MAVLink XML 规范](https://mavlink.io/en/guide/define_xml_element.html)，无额外私有帧头或自制 CRC。

## 消息与单位

| ID | 消息 | 频率 / 用途 |
| --- | --- | --- |
| 0 | HEARTBEAT | 1 Hz 在线提示 |
| 111 | TIMESYNC | 1 Hz 时钟同步，`tc1` / `ts1` 为 ns |
| 60000 | FINS_CAPABILITIES | 1 Hz 请求 / 应答，版本、会话、模式位、特性和模拟标志 |
| 60001 | FINS_STATUS | 模拟器 10 Hz，实际模式、电机、控制代次、故障、定深与灯光 |
| 60002 | FINS_ODOMETRY | 50 Hz，源时间、采样序号、重置编号、世界位姿 / 速度、深度与有效性 |
| 60003 | FINS_CONTROL_SETPOINT | 50 Hz，代次 / 序号 / TTL、归一化机体指令和垂直模式 |
| 60004 | FINS_ACTION_REQUEST | 可靠动作请求，绝对目标值 |
| 60005 | FINS_ACTION_ACK | 匹配请求编号 / 代次 / 动作，结果与拒绝原因 |
| 60006 | FINS_AUX_SETPOINT | 50 Hz，云台水平 / 俯仰 −1、0、1 |
| 60007 | FINS_TELEMETRY | 模拟器 50 Hz，电压、四路压力、八路 RPM、IMU 和三项网络预测 |

机体为 FLU：前、左、上为正；世界 XY 是 NX 局部里程计坐标。yaw 逆时针为正、单位 rad；角速度 rad/s，距离 m，速度 m/s。`depth` 独立向下为正。`forward/left/yaw` 在 [−1,1]，`up/down` 在 [0,1]；垂直手动输出为 up/down 两通道。定深模式只解释 `depth_target`，由 NX / 嵌入式执行垂直闭环。

`source_us` 为 NX 单调时钟微秒。PC 发送命令前用 TIMESYNC 把自己的单调时钟转换到 NX 时钟；不发送 UNIX 时间作为有效期基准。PC 只接受匹配请求、往返 ≤200 ms 的同步响应，用中点估算偏移；3 s 内无有效同步则撤销运动。该方法假设近似对称链路，实机必须校核时钟误差与链路延迟。

能力位：`mode_mask` 的 bit 0/1/2 分别为停止/手动/轨迹；PC 广播 0。`features` bit 0 为定深、bit 1 为辅助控制。遥测 `valid_mask` bit 0…4 为电压、压力、RPM、IMU、网络预测。无效字段不用于控制或有效样本。

所有数值宽度、数组长度、枚举和值域说明以 XML 为准。`FINS_REASON` 区分过时请求、无效代次、拒绝切换、无效值。模式和动作 Python 枚举引用生成值，避免手工维护第二份消息常量。

## 连接与控制权

默认 PC `0.0.0.0:54321`、NX `192.168.0.2:54322`；两端只接受配置的对端 IP 和端口。模拟默认使用回环地址。网络视频完全独立。

每次 PC 连接、NX 启动生成新的非零随机会话。PC 在协议版本与能力确认、时间同步后才可进入模式；重连仍需明确选择模式。所有动作 / 设定值包含发送会话和目标会话；接收方核对两者。模拟 NX 接收到新的 PC 会话会清除指令、模式和电机启用状态。

MAVLink CRC 检测损坏，不提供发送者身份认证。本阶段未实现 MAVLink 签名或多控制站抢占策略，适用于可信实验网络；后续远程多用户部署需另行设计。

## 模式与动作

1. 发送旧代次的零输出，退出旧模式。
2. 单调递增 `epoch`，发送 SET_MODE 请求（模式选择与启用电机独立）。
3. 收到会话、请求编号、动作和代次完全匹配的 ACCEPTED 后，重核输入有效性并进入新模式。
4. 被拒绝或超时保持停止，晚到 / 重复 ACK 不再次启动模式。

300 ms 无确认时重发同一请求，最多重试三次（共最多四次发送）。接收端按照 `(session,request_id)` 缓存结果，重复请求只重复 ACK，不再次执行；每类动作另保留最新请求号以拒绝乱序回放，不能用重复“切换”实现启停。模拟器保存最近 128 个确认以及每类动作的编号上界。

电机停用在新模式确认仍在途时也应接受，避免旧代次使停用失效。PC 同时撤销待确认模式激活。停止运动申请新的 IDLE 代次；定位 / 输入故障立即清零输出并锁存本地停止状态，NX 原模式可能尚未变更，界面分别显示。任何恢复都需要新的明确操作。

## 接收端有效期（后续 NX / 嵌入式必须实现）

- 验证当前 PC / NX 会话、已接受模式代次、有限数值和值域。
- 每路设定值使用 uint32 环形序号；重复、旧序号、旧代次一律丢弃。
- `valid_ms` 必须位于 1…200；源时刻最多允许超前 50 ms。抵达时已过期的报文丢弃。
- 截止时刻是 `min(接收时刻 + 200 ms, source_us + valid_ms)`，不能收到延迟包后再重新延长 200 ms。
- 截止后清空机体与定深输出，进入停止并上报故障；云台亦独立过期。新鲜指令不得自动恢复已停止模式，需新的模式请求。
- 里程计 `reset_id` 变化终止当前轨迹；下一次启动重新捕获原点。

模拟器是这些接收规则的可运行参考。PC 同时检查 NX 源时间及接收时间，任一超过 200 ms 均终止自动模式。动作请求编号 / 代次在应用运行期间递增；达到 uint32 上限前应建立新会话。

## 生成与验证

```powershell
uv run python tools/generate_protocol.py
$env:CC = 'C:\path\to\gcc.exe'
uv run python -m unittest tests.test_protocol -v
```

[test_vectors.json](../protocol/test_vectors.json) 固定 10 类消息的字段和完整帧，包含 MAVLink 2 消息 ID 与 CRC。测试分别用 Python 编解码和本机编译的 C 编解码检查同一向量，并注入 CRC 损坏。改变字段后需同步版本与测试向量，并重新评审对端兼容性。
