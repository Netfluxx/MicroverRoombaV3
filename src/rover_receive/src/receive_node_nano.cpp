#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <SerialStream.h> // Include the SerialStream header file

class ReceiveNode : public rclcpp::Node {
public:
  ReceiveNode() : Node("receive_node") {
    // Open serial port (replace "/dev/ttyUSB0" with your Arduino's port)
    serial_port_ = new LibSerial::SerialStream("/dev/ttyUSB0", LibSerial::SerialStreamBuf::BAUD_9600);
    if (!serial_port_->good()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      throw std::runtime_error("Failed to open serial port");
    }

    // Subscribe to the input topic
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "input_topic", 10, std::bind(&ReceiveNode::messageCallback, this, std::placeholders::_1));
  }

  ~ReceiveNode() {
    delete serial_port_;
  }

private:
  void messageCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received by RPi: %s", msg->data.c_str());

    // Send the message over the serial port
    (*serial_port_) << msg->data << std::endl; // Send with newline
    RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  LibSerial::SerialStream* serial_port_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReceiveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
