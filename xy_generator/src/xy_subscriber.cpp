#include "rclcpp/rclcpp.hpp"
#include "xy_generator/msg/xy.hpp" // Custom message header

class XYSubscriber : public rclcpp::Node
{
public:
  XYSubscriber()
  : Node("xy_subscriber")
  {
    subscription_ = this->create_subscription<xy_generator::msg::XY>(
      "xy_topic", 10, std::bind(&XYSubscriber::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "XY Subscriber has been started.");
  }

private:
  void topic_callback(const xy_generator::msg::XY::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received: x=%.2f, y=%.2f", msg->x, msg->y);
  }

  rclcpp::Subscription<xy_generator::msg::XY>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XYSubscriber>());
  rclcpp::shutdown();
  return 0;
}