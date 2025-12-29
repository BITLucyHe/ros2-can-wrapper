# ROS2 CAN Wrapper

English | [简体中文](./README_CN.md)

A ROS2 node wrapper based on SocketCAN.

Tested only on ROS2 Humble. Theoretically compatible with other ROS2 distributions.

Parameters in `can_params.yaml`:

- `can_interface`: CAN interface name (e.g., `can0`, `vcan0`).
- `can_filter_id`: Receive filter ID. The node only processes CAN frames matching this ID.
- `can_send_id`: Send ID. CAN frames sent by this node will use this ID.

Modify `include/can/protocol.hpp` to customize message packets.
