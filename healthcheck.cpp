#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "cstdlib"

using namespace std::chrono_literals;

#define TOPIC_NAME "/camera/color/image_raw"
#define TIMEOUT 2s

int msg_received = EXIT_FAILURE;

void msg_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::cout << "Message received" << std::endl;
    msg_received = EXIT_SUCCESS;
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
  auto node = rclcpp::Node::make_shared("listener_scan");
  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(TOPIC_NAME, 10, msg_callback);

  // Set a timer for the timeout duration (e.g., 5 seconds)
  auto timer = node->create_wall_timer(TIMEOUT, timeout_callback);

  rclcpp::spin(node);
  return msg_received;
}
