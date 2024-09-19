#include <chrono>
#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "xy_generator/msg/xy.hpp" // Custom message header

using namespace std::chrono_literals;

class XYPublisher : public rclcpp::Node
{
public:
  XYPublisher()
  : Node("xy_publisher")
  {
    publisher_ = this->create_publisher<xy_generator::msg::XY>("xy_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&XYPublisher::publish_xy, this));
    RCLCPP_INFO(this->get_logger(), "XY Publisher has been started.");
  }

private:
  void publish_xy()
  {
    auto message = xy_generator::msg::XY();
    message.x = generate_random();
    message.y = generate_random();
    RCLCPP_INFO(this->get_logger(), "Publishing: x=%.2f, y=%.2f", message.x, message.y);
    publisher_->publish(message);
  }

  float generate_random()
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 100.0);
    return dis(gen);
  }

  rclcpp::Publisher<xy_generator::msg::XY>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XYPublisher>());
  rclcpp::shutdown();
  return 0;
}