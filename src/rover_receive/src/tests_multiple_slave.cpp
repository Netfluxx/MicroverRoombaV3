#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <SerialStream.h>

class ReceiveNode : public rclcpp::Node {
public:
  ReceiveNode() : Node("receive_node") {
    // Open serial port (replace "/dev/ttyUSB0" with your Arduino's port)
    serial_port_ = new LibSerial::SerialStream("/dev/ttyUSB0", LibSerial::BaudRate::BAUD_9600);

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
    // Send the message over the serial port
    (*serial_port_) << msg->data << std::endl; // Send with newline

    // Read data from the serial port (what Arduino sends back)
    std::string received_from_arduino;
    std::getline(*serial_port_, received_from_arduino);
    
    if (received_from_arduino.empty()) {
      RCLCPP_WARN(this->get_logger(), "No data received from Arduino");
      return;
    }

    // Parse the received string
    parseFeedback(received_from_arduino);
  }

  void parseFeedback(const std::string &feedback) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(feedback);

    while (std::getline(tokenStream, token, ',')) {
      tokens.push_back(token);
    }

    if (tokens.size() != 13) { // 1 command + 3 * 4 (send, fb, speed for each of the 4 motors)
      RCLCPP_ERROR(this->get_logger(), "Unexpected feedback format: %s", feedback.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received feedback:");
    RCLCPP_INFO(this->get_logger(), "Received Command: %s", tokens[0].c_str());

    for (int i = 0; i < 4; ++i) {
      RCLCPP_INFO(this->get_logger(), "Motor %d Send Status: %s", i+1, tokens[1 + i * 3].c_str());
      RCLCPP_INFO(this->get_logger(), "Motor %d Feedback Status: %s", i+1, tokens[2 + i * 3].c_str());
      RCLCPP_INFO(this->get_logger(), "Motor %d Speed: %s", i+1, tokens[3 + i * 3].c_str());
    }

    std::cout << "----------------" << std::endl;
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
