# ROS_CRSF_Bridge

PX4固件支持： [https://docs.px4.io/main/zh/telemetry/crsf_telemetry](https://docs.px4.io/main/zh/telemetry/crsf_telemetry)

ROS1 节点：订阅 `sensor_msgs/Joy`，将 `axes[]` 映射为 **CRSF** (TBS Crossfire) 的 **RC_CHANNELS_PACKED (0x16)** 帧，通过串口发送至飞控。

同时支持 **CRSF 遥测回传**：从同一串口接收 CRSF 遥测帧并发布为 ROS topics（Hybrid：**原始帧 + 解析后的标准消息**）。

包名：`crsf_control`；节点/可执行程序：`crsf_control_node`。

## 功能

- 订阅 `sensor_msgs/Joy`，将 `axes[]` 映射到 CRSF 16 通道
- 打包为 `CRSF_FRAMETYPE_RC_CHANNELS_PACKED` (0x16) 帧，经串口输出
- 独立发送线程，保证帧发送频率稳定
- **遥测接收（新增）**：串口 RX 流式拆帧 + CRC 校验，发布 `~telemetry/raw` 原始帧
- **遥测解析发布（新增）**（首批支持）：
  - GPS (0x02) → `sensor_msgs/NavSatFix`
  - Battery (0x08) → `sensor_msgs/BatteryState`
  - Link Statistics (0x14) → `crsf_control/CrsfLinkStatistics`
  - Attitude (0x1E) → `sensor_msgs/Imu`（roll/pitch/yaw → quaternion）
  - Flight Mode (0x21) → `std_msgs/String`
- 安全监控：Joy 消息频率过低报警，超时自动断开 CRSF 输出
- 超时恢复后自动重新启用输出
- 所有参数通过 ROS Parameter 配置

## CRSF 协议概要

当前实现使用的 CRSF 基本帧格式（接收侧以 `0xC8` 作为同步字节）：

```
帧格式: [ADDR/SYNC:0xC8] [LEN] [TYPE] [PAYLOAD...] [CRC8]
CRC: CRC8，poly=0xD5，范围 TYPE → PAYLOAD 末尾
串口: 420000 baud, 8N1
```

RC 通道帧（0x16）：

```
[0xC8] [LEN:0x18] [TYPE:0x16] [PAYLOAD:22B] [CRC8]   # 总长 26 字节
```

通道值域：

| CRSF 值 | PWM (us) | Joy axes |
|---------|----------|----------|
| 172     | 988      | -1.0     |
| 992     | 1500     | 0.0      |
| 1811    | 2012     | 1.0      |

## 编译

> 注：当前 `CMakeLists.txt` 使用 **C++17**。

```bash
# 将本包放入 catkin workspace 的 src/ 目录下
cp -r crsf_control ~/catkin_ws/src/

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行

```bash
roslaunch crsf_control crsf_control.launch
```

或使用自定义参数：

```bash
roslaunch crsf_control crsf_control.launch \
  _serial_port:=/dev/ttyACM0 \
  _crsf_rate:=100 \
  _timeout:=1.0
```

### 遥测 Topics（~telemetry/*）

节点会在私有命名空间 `~` 下发布以下 topics：

| Topic | ROS 类型 | 说明 |
|---|---|---|
| `~telemetry/raw` | `crsf_control/CrsfFrame` | 校验通过的原始 CRSF 帧（结构化字段） |
| `~telemetry/battery` | `sensor_msgs/BatteryState` | 电压/电流/电量百分比 |
| `~telemetry/gps` | `sensor_msgs/NavSatFix` | 经纬度/高度 |
| `~telemetry/imu` | `sensor_msgs/Imu` | 姿态 quaternion（协方差未知时置 -1） |
| `~telemetry/flight_mode` | `std_msgs/String` | 飞行模式字符串 |
| `~telemetry/link_statistics` | `crsf_control/CrsfLinkStatistics` | 链路质量/信号强度等 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `joy_topic` | string | `/joy` | 订阅的 Joy 消息 topic |
| `serial_port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `serial_baud` | int | `420000` | 串口波特率 |
| `crsf_rate` | int | `50` | CRSF 帧发送频率 (Hz, 1-500) |
| `min_frequency` | double | `10.0` | Joy 消息最低频率阈值 (Hz)，低于此值报警 |
| `timeout` | double | `0.5` | Joy 消息超时阈值 (秒)，超时后停止 CRSF 帧输出 |

## 架构

数据流（发送 + 接收）：

```
# RC 发送
sensor_msgs/Joy ──► joyCallback() ──► channels_[] (mutex)
                                          │
                  senderLoop() (独立线程) ◄─┘
                      │
                      ├── 超时检测 → 停止输出
                      ├── 频率检测 → ROS_WARN
                      ├── packChannelsFrame()
                      └── SerialPort::write() → 飞控

# 遥测接收
SerialPort::waitReadable/read() ──► FrameStreamParser (拆帧+CRC)
                                 ├──► ~telemetry/raw (CrsfFrame)
                                 └──► TelemetryDecoder ──► ~telemetry/* (Battery/GPS/IMU/FlightMode/LinkStats)
```

核心类：

| 类 | 职责 |
|----|------|
| `CrsfProtocol` | CRC8 计算、通道值转换、CRSF 帧打包 |
| `SerialPort` | Linux 串口封装 (termios2 + BOTHER 自定义波特率)；提供 read/poll |
| `FrameStreamParser` | CRSF 流式拆帧（长度约束 + CRC 校验 + 滑窗重同步） |
| `TelemetryDecoder` | CRSF 遥测 payload 解析（GPS/Battery/LinkStats/Attitude/FlightMode） |
| `CrsfNode` | ROS 节点、Joy 回调、发送线程 + 遥测接收线程、ROS 发布 |

## 自定义 ROS 消息

### `crsf_control/CrsfFrame`

结构化承载校验通过的原始 CRSF 帧：

- `std_msgs/Header header`
- `uint8 device_address`
- `uint8 frame_length`
- `uint8 type`
- `uint8[] payload`
- `uint8 crc`

### `crsf_control/CrsfLinkStatistics`

对应 CRSF Link Statistics (0x14) 的常用字段（RSSI 为原始 u8，常见含义为 `dBm = -value`）：

- uplink：`uplink_rssi_1/uplink_rssi_2/uplink_link_quality/uplink_snr/active_antenna/rf_mode/uplink_tx_power`
- downlink：`downlink_rssi/downlink_link_quality/downlink_snr`

## 文件结构

```
crsf_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── msg/
│   ├── CrsfFrame.msg
│   └── CrsfLinkStatistics.msg
├── launch/
│   └── crsf_control.launch
├── include/crsf_control/
│   ├── crsf_protocol.h
│   ├── serial_port.h
│   ├── crsf_frame_stream_parser.h
│   ├── crsf_telemetry_decoder.h
│   └── crsf_node.h
└── src/
    ├── crsf_protocol.cpp
    ├── serial_port.cpp
    ├── crsf_frame_stream_parser.cpp
    ├── crsf_telemetry_decoder.cpp
    ├── crsf_node.cpp
    └── main.cpp
```

## 安全机制

1. **频率监控**：基于指数移动平均估算 Joy 消息频率，低于 `min_frequency` 时 `ROS_WARN` 报警
2. **超时保护**：Joy 消息间隔超过 `timeout` 秒时 `ROS_ERROR` 报警并停止 CRSF 帧输出
3. **自动恢复**：Joy 消息恢复后自动重新启用输出并记录日志
4. **串口错误**：写入失败时记录错误计数，不会阻塞发送线程
5. **可退出的接收线程**：接收侧使用 `poll` 超时轮询，便于 shutdown 时及时退出

## 依赖

- ROS1 (Noetic / Melodic)
- roscpp
- sensor_msgs
- std_msgs
- message_generation / message_runtime（用于自定义 msg）
- Linux (termios2, asm/termbits.h)

## 硬件接线

### 仅 RC 单向发送（不需要遥测）

- 主机/伴随计算机 **TX** → 飞控 UART **RX**
- GND 互联

### RC + 遥测回传（需要遥测）

必须连接双向串口：

- 主机/伴随计算机 **TX** → 飞控 UART **RX**
- 主机/伴随计算机 **RX** ← 飞控 UART **TX**
- GND 互联

> 说明：飞控侧需要将对应 UART 配置为 CRSF（具体方法依飞控固件而定）。

## 协议参考

- [TBS 官方 CRSF 规范](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md)
- [CRSF Working Group Wiki](https://github.com/crsf-wg/crsf/wiki)
- [RC_CHANNELS_PACKED 帧详情](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_RC_CHANNELS_PACKED)

## 许可证

MIT
