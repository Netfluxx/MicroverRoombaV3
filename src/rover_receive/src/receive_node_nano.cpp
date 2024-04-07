#include "rclcpp/rclcpp.hpp"  
#include "std_msgs/msg/string.hpp"  
#include <wiringPi.h>  
#include <wiringSerial.h> // For serial communication using WiringPi

class ReceiveNode : public rclcpp::Node {  
public:  
ReceiveNode() : Node("receive_node") {  
// Initialize WiringPi  
if (wiringPiSetup() == -1) {  
RCLCPP_ERROR(this->get_logger(), "Failed to initialize WiringPi");  
throw std::runtime_error("Failed to initialize WiringPi");  
}

// Open serial port (replace "/dev/ttyUSB0" with your Arduino's port)  
serial_port_ = serialOpen("/dev/ttyUSB0", 9600);  
if (serial_port_ == -1) {  
RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");  
throw std::runtime_error("Failed to open serial port");  
}

// Subscribe to the input topic  
subscription_ = this->create_subscription<std_msgs::msg::String>(  
"input_topic", 10, std::bind(&ReceiveNode::messageCallback, this, std::placeholders::_1));  
}

private:  
void messageCallback(const std_msgs::msg::String::SharedPtr msg) {  
// Append a newline character to the message data  
std::string message_with_newline = msg->data + "\n";

// Log the received message  
RCLCPP_INFO(this->get_logger(), "Received by RPi: %s", msg->data.c_str());

// Send the message over the serial port  
serialPrintf(serial_port_, "%s", message_with_newline.c_str());

// Log the message sent to the Arduino  
RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", msg->data.c_str());  
}

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  
int serial_port_;  
};

int main(int argc, char** argv) {  
rclcpp::init(argc, argv);  
auto node = std::make_shared<ReceiveNode>();  
rclcpp::spin(node);  
rclcpp::shutdown();  
return 0;  
}
