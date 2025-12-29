# ROS2 CAN Wrapper

[English](./README.md) | 简体中文

这是一个基于 SocketCAN 的 ROS2 节点封装。

仅在 ROS2 Humble 环境下测试通过。理论上兼容其他 ROS2 发行版。

`can_params.yaml` 文件中参数含义如下：

- `can_interface`: CAN 设备接口名称（如 `can0`, `vcan0`）。
- `can_filter_id`: 接收过滤器 ID。节点只处理 ID 匹配此值的 CAN 帧。
- `can_send_id`: 发送 ID。节点发出的 CAN 帧将使用此 ID。

修改 `include/can/protocol.hpp` 文件以自定义消息包。
