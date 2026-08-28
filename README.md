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

点击“开始采集”选择保存路径。CSV 使用 UTF-8 BOM，记录每个有效数据包的主机接收时间、来源、全部遥测字段、时间戳间隔及连续性结果。

如果 AprilTag 定位正在运行，每一条 NX 遥测行还会冻结并记录当时的最新视觉快照：视觉帧 UTC 时间、帧龄、有效性、`vision_x_m`、`vision_y_m`、`vision_yaw_rad`、可见 Tag ID、数量、重投影误差与状态。相机帧率通常低于 200 Hz，因此相邻遥测行可能对应同一视觉帧；`vision_age_ms` 和 `vision_valid` 用于判断该快照是否新鲜。点击“停止采集”后程序会清空写入队列并关闭文件。

## AprilTag 全局定位

界面中的“扫描相机”会在后台探测 Windows 相机索引 0–7；选择索引后点击“启动定位”。定位使用 `tag25h9` 中的 ID 10、11、12、13，并在画面显示识别框、ID、机体系和实时位置。

- 相机内参：`D:\fins\tools\finsrov_perception\calibration\rgb_camera.yaml`，只接受对应的 **1280×720** 图像，避免未标定缩放引入尺度错误。
- Tag 几何：`apriltag_layout.json`。黑色边框间距为 **0.093 m**；相邻 Tag 的中心距为 **0.140 m**。ID 10、13 一侧为 ROV 前方。
- 坐标：相机参考系投影到 Tag 平面，`+X` 向画面右、`+Y` 向画面上，yaw 从 `+X` 逆时针为正。定位只显示和写入 CSV，不会写入 A 板的 52 字节控制帧。

若实际打印的 Tag 朝向不同，可在 `apriltag_layout.json` 中调整各 ID 的 `yaw_deg`；此值定义该 Tag 印刷顶部相对于 ROV 前方的方向。

定位数据在进入 CSV 前会直接丢弃坏视觉帧：四 Tag 同时可见时，16 个角点必须通过 `6 px` 几何重投影误差检查；相邻可靠视觉帧的速度不得超过 `0.8 m/s`、航向角速度不得超过 `3.0 rad/s`。被丢弃帧不会更新定位缓存。

CSV 写入端会等待可靠视觉帧的前后边界，并按每条 200 Hz NX 遥测的主机接收时间，对 `X/Y` 做线性插值、对 yaw 做最短角度线性插值。只有处在两帧可靠视觉结果之间、且视觉间隔不超过 150 ms 的遥测行才会写入 CSV；开始定位、视觉丢失、坏帧或采集停止时没有可插值定位的遥测行会直接丢弃。界面会显示“定位未插值丢弃”计数。

## 键盘控制

上位机默认每 20 ms 向 NX `192.168.0.2:54322` 发送一个 CRC16 校验过的 52 字节 `Streamer` 控制帧。

- `↑/↓`：前进/后退
- `←/→`：左移/右移
- `A/D`：逆时针/顺时针偏航
- “电机启停”按钮：仅发送一次 `startButton` 的 `1 → 0` 脉冲

控制区可配置 NX IP、控制端口、平移幅值和偏航幅值。松开按键或窗口失焦会立即发送零运动指令；NX 在 200 ms 未收到合法控制帧时也会持续下发零指令。

## NX 串口转 UDP

`nx_serial_to_udp.py` 已被 `D:\fins\tools\NXStreamer\nx_streamer.py` 取代。部署时只运行新的双向中转程序，避免多个进程同时占用 `/dev/ttyTHS1`。

```bash
scp nx_serial_to_udp.py <nx-user>@192.168.0.2:~/
ssh <nx-user>@192.168.0.2
python3 ~/nx_serial_to_udp.py --udp-ip <运行上位机的PC-IP> --udp-port 54321 --baud 115200
```

`--udp-ip` 必须填写运行本上位机的 **PC IP**，不是 NX 自身 IP。若 STM32 改为更高串口波特率，可用 `--baud 230400` 等参数同步修改；支持 9600 至 921600 的常用档位。
