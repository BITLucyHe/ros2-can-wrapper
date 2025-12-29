#include "can/can_node.hpp"

using namespace std::chrono_literals;


CanNode::CanNode(const rclcpp::NodeOptions &options)
: Node("can_node", options) {
    can_interface = this->declare_parameter("can_interface", "can0");
    can_filter_id = this->declare_parameter("can_filter_id", 0x32);
    can_send_id = this->declare_parameter("can_send_id", 0x87);

    can_socket = std::make_shared<CanSocket>(can_interface.data(), can_filter_id, can_send_id);

    track_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/any_topic1", 1,
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
            this->send_to_slave(msg);
        }
    );

    slave_pub_ = this->create_publisher<std_msgs::msg::Int8>("/any_topic2", rclcpp::SensorDataQoS());
    receive_timer_ = this->create_wall_timer(
        1ms,
        [this]() { this->receive_from_slave(); }
    );
}

void CanNode::send_to_slave(const std_msgs::msg::Int8::SharedPtr ptr) {
    msg_send tmp {
        static_cast<uint8_t>(ptr->data),
    };

    this->can_socket->send(tmp);
}

void CanNode::receive_from_slave() {
    if (auto packet = this->can_socket->receive()) {
        std_msgs::msg::Int8 tmp;
        tmp.set__data(packet->any_data);

        slave_pub_->publish(tmp);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CanNode)
