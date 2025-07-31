#include "bitbot_gz/kernel/gz_kernel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "user_func.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::shutdown();
  return 0;
}
