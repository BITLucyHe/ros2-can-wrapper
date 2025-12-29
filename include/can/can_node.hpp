#pragma once

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

#include "CanSocket.hpp"
#include "protocol.hpp"


class CanNode : public rclcpp::Node {

public:
    CanNode(const rclcpp::NodeOptions &options);
    ~CanNode() = default;

private:
    void send_to_slave(const std_msgs::msg::Int8::SharedPtr ptr);
    void receive_from_slave();

    std::string can_interface;
    int can_filter_id;
    int can_send_id;
    std::shared_ptr<CanSocket> can_socket;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr track_sub_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr slave_pub_;
    rclcpp::TimerBase::SharedPtr receive_timer_;

};
