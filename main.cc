#include "bitbot_gz/kernel/gz_kernel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "user_func.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  bitbot::GzKernel<int, "1">* ptr;
  rclcpp::shutdown();

  return 0;
}
