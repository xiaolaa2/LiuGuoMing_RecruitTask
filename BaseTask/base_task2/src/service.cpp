#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "base_task2/srv/transform_service2.hpp"
#include <memory>

void add(const std::shared_ptr<base_task2::srv::TransformService2::Request> request,
          std::shared_ptr<base_task2::srv::TransformService2::Response> response) {
  auto x = request->x;
  auto y = request->y;
  // 坐标差
  auto x2 = request->x2;
  auto y2 = request->y2;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "original point is (%d, %d)", request->x, request->y);

  int a = x + x2;
  int b = y + y2;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "target point is (%d, %d)", a, b);

  response->a = a;
  response->b = b;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("my_transform_service");

  rclcpp::Service<base_task2::srv::TransformService2>::SharedPtr service =
    node->create_service<base_task2::srv::TransformService2>("my_transform_service", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "正在进行坐标转换");

  rclcpp::spin(node);
  rclcpp::shutdown();
}