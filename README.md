# ROS_CRSF_Bridge

ROS1 节点：订阅 `sensor_msgs/Joy` 消息，转换为 CRSF (TBS Crossfire) RC 通道帧，通过串口发送至飞控。

## 功能

- 订阅 `sensor_msgs/Joy`，将 `axes[]` 映射到 CRSF 16 通道
- 打包为 `CRSF_FRAMETYPE_RC_CHANNELS_PACKED` (0x16) 帧，经串口输出
- 独立发送线程，保证帧发送频率稳定
- 安全监控：Joy 消息频率过低报警，超时自动断开 CRSF 输出
- 超时恢复后自动重新启用输出
- 所有参数通过 ROS Parameter 配置

## CRSF 协议概要

```
帧格式: [SYNC:0xC8] [LEN:0x18] [TYPE:0x16] [PAYLOAD:22B] [CRC8]
通道数: 16
通道位宽: 11 bit，LSB-first，little-endian
帧总长: 26 字节
CRC: CRC8，poly=0xD5，范围 TYPE → PAYLOAD 末尾
串口: 420000 baud, 8N1, Full Duplex
```

通道值域：

| CRSF 值 | PWM (us) | Joy axes |
|---------|----------|----------|
| 172     | 988      | -1.0     |
| 992     | 1500     | 0.0      |
| 1811    | 2012     | 1.0      |

## 编译

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

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `joy_topic` | string | `/joy` | 订阅的 Joy 消息 topic |
| `serial_port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `serial_baud` | int | `420000` | 串口波特率 |
| `crsf_rate` | int | `50` | CRSF 帧发送频率 (Hz, 1-500) |
| `min_frequency` | double | `10.0` | Joy 消息最低频率阈值 (Hz)，低于此值报警 |
| `timeout` | double | `0.5` | Joy 消息超时阈值 (秒)，超时后停止 CRSF 输出 |

## 架构

```
sensor_msgs/Joy ──► joyCallback() ──► channels_[] (mutex)
                                          │
                  senderLoop() (独立线程) ◄─┘
                      │
                      ├── 超时检测 → 停止输出
                      ├── 频率检测 → ROS_WARN
                      ├── packChannelsFrame()
                      └── serial write → 飞控
```

核心类：

| 类 | 职责 |
|----|------|
| `CrsfProtocol` | CRC8 计算、通道值转换、CRSF 帧打包 |
| `SerialPort` | Linux 串口封装 (termios2 + BOTHER 自定义波特率) |
| `CrsfNode` | ROS 节点、Joy 回调、发送线程、安全监控 |

## 文件结构

```
crsf_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── crsf_control.launch
├── include/crsf_control/
│   ├── crsf_protocol.h
│   ├── serial_port.h
│   └── crsf_node.h
└── src/
    ├── crsf_protocol.cpp
    ├── serial_port.cpp
    ├── crsf_node.cpp
    └── main.cpp
```

## 安全机制

1. **频率监控**：基于指数移动平均估算 Joy 消息频率，低于 `min_frequency` 时 `ROS_WARN` 报警
2. **超时保护**：Joy 消息间隔超过 `timeout` 秒时 `ROS_ERROR` 报警并停止 CRSF 帧输出
3. **自动恢复**：Joy 消息恢复后自动重新启用输出并记录日志
4. **串口错误**：写入失败时记录错误计数，不会阻塞发送线程

## 依赖

- ROS1 (Noetic / Melodic)
- roscpp
- sensor_msgs
- Linux (termios2, asm/termbits.h)

## 硬件接线

将串口 TX 连接至飞控的 CRSF RX 引脚（如 PX4 的 TELEM/SERIAL 端口）。仅需 TX 线（单向发送，不接收遥测）。

## 协议参考

- [TBS 官方 CRSF 规范](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md)
- [CRSF Working Group Wiki](https://github.com/crsf-wg/crsf/wiki)
- [RC_CHANNELS_PACKED 帧详情](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_RC_CHANNELS_PACKED)

## 许可证

MIT
