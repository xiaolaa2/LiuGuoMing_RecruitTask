#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

    std::string currentDateTime() {

      // 获取当前时间点
      auto now = std::chrono::system_clock::now();

      // 把时间点转换为时间戳
      auto timestamp = std::chrono::system_clock::to_time_t(now);

      // 把时间戳转换为本地时间
      struct tm* local_time = localtime(&timestamp); 

      // 定义格式化的格式
      char formatted[100];

      // 格式化本地时间
      strftime(formatted, sizeof(formatted), "%Y-%m-%d %H:%M:%S", local_time);

      return std::string(formatted); 
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = currentDateTime();
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}