#include "fstream"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

#define HEALTHCHECK_PERIOD 500ms
#define MSG_VALID_TIME 2s

class HealthCheckNode : public rclcpp::Node {
public:
  HealthCheckNode()
      : Node("healthcheck_astra"),
        last_msg_time(std::chrono::steady_clock::now()) {

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "color/image_raw", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&HealthCheckNode::msgCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(HEALTHCHECK_PERIOD,
                               std::bind(&HealthCheckNode::healthyCheck, this));
  }

private:
  std::chrono::steady_clock::time_point last_msg_time;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  void writeHealthStatus(const std::string &status) {
    std::ofstream healthFile("/var/tmp/health_status.txt");
    healthFile << status;
  }

  void msgCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Msg arrived");
    last_msg_time = std::chrono::steady_clock::now();
  }

  void healthyCheck() {
    std::chrono::steady_clock::time_point current_time =
        std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_msg_time;
    bool is_msg_valid = elapsed_time < MSG_VALID_TIME;

    if (is_msg_valid) {
      writeHealthStatus("healthy");
    } else {
      writeHealthStatus("unhealthy");
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthCheckNode>());
  rclcpp::shutdown();
  return 0;
}
