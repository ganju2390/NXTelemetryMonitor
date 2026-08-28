# NX Telemetry Monitor

用于接收 FineSUB 由 NX 转发的 UDP 遥测数据，并实时显示 8 路推进器转速。

## 环境与启动

本项目使用 `uv` 管理 Python 3.12 与隔离的 `.venv`。首次使用时打开 PowerShell：

```powershell
cd D:\fins\tools\NXTelemetryMonitor
uv sync
uv run python nx_telemetry_monitor.py
```

也可在终端重启、`uv` 已位于 PATH 后双击 `run_monitor.bat`。

`uv sync` 会安装 OpenCV contrib、NumPy、Pillow 与 PyYAML；Tkinter 随 uv 管理的 CPython 一起提供。

## UDP 配置

- **NX 发送方 IP**：发送遥测数据的 NX IP。留空可接受任意来源。
- **监听端口**：本机绑定的 UDP 端口，应用监听后从 `0.0.0.0:<端口>` 接收。
- 修改配置后点击“应用监听配置”；会重启 UDP socket 并重新开始时间戳校验。

## 固件数据协议

UDP datagram 必须原样承载 `V5_SUB::UploadData`，共 48 字节、小端、紧凑布局：

| 字节范围 | 字段 |
| --- | --- |
| 0–23 | `float pressure, accX, accY, accZ, gyroZ, yaw` |
| 24–39 | `int16_t intRpm1` 到 `int16_t intRpm8` |
| 40–43 | `float timeStamp` |
| 44–47 | 尾标 `00 00 80 7F` |

时间戳期望相邻数据包增加 `0.005 s`。程序以 `±0.0005 s` 判定连续性，并统计异常和推断丢帧数。

## CSV 采集

点击“开始采集”选择保存路径。CSV 使用 UTF-8 BOM，仅记录 NX 遥测数据（压力、IMU、8 路 RPM、固件时间戳）与 AprilTag 定位数据。

每条写入行含 `received_utc`、`firmware_timestamp`、`tag_timestamp_utc`、`tag_x_m`、`tag_y_m`、`tag_yaw_rad`；Tag 位姿按同一条 NX 遥测的主机接收时刻插值对齐。来源地址、连续性诊断、Tag ID、重投影误差与状态等调试字段不会写入 CSV。

## AprilTag 全局定位

界面中的“扫描相机”会在后台探测 Windows 相机索引 0–7；选择索引后点击“启动定位”。定位使用 `tag25h9` 中的 ID 10、11、12、13，并在画面显示识别框、ID、机体系和实时位置。

- 相机内参：`D:\fins\tools\finsrov_perception\calibration\rgb_camera.yaml`，只接受对应的 **1280×720** 图像，避免未标定缩放引入尺度错误。
- Tag 几何：`apriltag_layout.json`。黑色边框间距为 **0.093 m**；相邻 Tag 的中心距为 **0.140 m**。ID 10、13 一侧为 ROV 前方。
- 坐标：相机参考系投影到 Tag 平面，`+X` 向画面右、`+Y` 向画面上，yaw 从 `+X` 逆时针为正。定位只显示和写入 CSV，不会写入 A 板的 52 字节控制帧。

若实际打印的 Tag 朝向不同，可在 `apriltag_layout.json` 中调整各 ID 的 `yaw_deg`；此值定义该 Tag 印刷顶部相对于 ROV 前方的方向。

定位数据在进入 CSV 前会直接丢弃坏视觉帧：四 Tag 同时可见时，16 个角点必须通过 `6 px` 几何重投影误差检查；相邻可靠视觉帧的速度不得超过 `0.8 m/s`、航向角速度不得超过 `3.0 rad/s`。被丢弃帧不会更新定位缓存。

CSV 写入端会等待可靠视觉帧的前后边界，并按每条 200 Hz NX 遥测的主机接收时间，对 `X/Y` 做线性插值、对 yaw 做最短角度线性插值。只要前后可靠视觉帧间隔不超过 500 ms，遥测行就会写入 CSV；开始定位、视觉丢失、坏帧或采集停止后仍没有可插值定位的遥测行会直接丢弃。停止采集时会额外等待最多 550 ms 以获得最后一帧的插值右边界。界面会显示“定位未插值丢弃”计数。

## 键盘控制

上位机默认每 20 ms 向 NX `192.168.0.2:54322` 发送一个 CRC16 校验过的 52 字节 `Streamer` 控制帧。

- `↑/↓`：前进/后退
- `←/→`：左移/右移
- `A/D`：逆时针/顺时针偏航
- “电机启停”按钮：仅发送一次 `startButton` 的 `1 → 0` 脉冲

控制区可配置 NX IP、控制端口、平移幅值和偏航幅值。松开按键或窗口失焦会立即发送零运动指令；NX 在 200 ms 未收到合法控制帧时也会持续下发零指令。

## CSV 轨迹追踪

轨迹区域可选择 `v1` 或 `v2`：v1 是动力学辨识集，v2 是论文 demo 集。默认选择 v2；采集文件会按已加载轨迹的版本写入 `dataset/v1/` 或 `dataset/v2/`，并保持与轨迹文件同名。CSV 使用 UTF-8，表头必须严格为：

```csv
x_m,y_m,yaw_rad,duration_s
0,0,0,4
0.5,0,0,4
```

首行必须是相机全局坐标的 `(0,0,yaw)`。`duration_s` 表示从上一关键点到当前行的设定时长；首行则表示从启动时机器实际位置到首点的时长。点击“启动轨迹”后，**首段严格从当前实际 XY 到 CSV 首个关键点**。每一段仅规划到当前关键点；只有 AprilTag XY 进入该点 **0.15 m** 半径且 yaw 最短角误差不超过 **10°** 后，才以实际当前位置建立下一段。每段使用 C2 三次样条，严格采用 CSV 的 `duration_s`，不再根据程序内的速度预设自动修改时长；界面显示的参考／实际 yaw 均归一化到 `[-π, π]`。v2 的每条 demo 轨迹由一个闭合图形重复两圈组成，共 21 个点、每段 6 s，设定时长为 126 s。

自动模式每 20 ms 发送与键盘完全相同的普通 52 字节控制帧：`enablePathTracking=0`，三个运动字段分别是归一化 `yawRate`、`forwardSpeed`、`leftRightSpeed`。它们均限制为 `[-1,1]`，并经过油门斜率限制。XY、yaw 与 yaw rate 闭环均使用已通过 AprilTag 几何与时间突变检查的视觉缓存；定位超时 200 ms、相机停止或无可靠位姿时会立即取消自动模式并发送零命令。

自动模式忽略方向键和 `A/D`；使用“停止轨迹”恢复手动控制。轨迹完成后保持最终位置／航向参考并输出零水平与 yaw 命令，直至停止。电机启停按钮独立，不会因启动轨迹而自动触发。

`training_trajectories/v1/manifest.csv` 是动力学辨识集的索引。`training_trajectories/v2/manifest.csv` 是论文 demo 集的索引，含椭圆、8 字、矩形、十字、半月牙与随机路径。使用 `python generate_training_trajectories.py --check` 或 `python generate_demo_trajectories_v2.py --check` 可只验证对应 catalog；不带参数会重新生成该版本的 CSV。

## NX 串口转 UDP

`nx_serial_to_udp.py` 已被 `D:\fins\tools\NXStreamer\nx_streamer.py` 取代。部署时只运行新的双向中转程序，避免多个进程同时占用 `/dev/ttyTHS1`。

```bash
scp nx_serial_to_udp.py <nx-user>@192.168.0.2:~/
ssh <nx-user>@192.168.0.2
python3 ~/nx_serial_to_udp.py --udp-ip <运行上位机的PC-IP> --udp-port 54321 --baud 115200
```

`--udp-ip` 必须填写运行本上位机的 **PC IP**，不是 NX 自身 IP。若 STM32 改为更高串口波特率，可用 `--baud 230400` 等参数同步修改；支持 9600 至 921600 的常用档位。
