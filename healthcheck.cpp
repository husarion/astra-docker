#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cstdlib"

using namespace std::chrono_literals;

#define TIMEOUT 2s

bool msg_received = false;

void msg_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::cout << "Message received" << std::endl;
  msg_received = true;
  rclcpp::shutdown();
}

void timeout_callback()
{
  std::cout << "Timeout" << std::endl;
  rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("healthcheck_node");
  auto sub = node->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", rclcpp::SensorDataQoS(), msg_callback);
  auto timer = node->create_wall_timer(TIMEOUT, timeout_callback);

  rclcpp::spin(node);
  return msg_received ? EXIT_SUCCESS : EXIT_FAILURE;
}
