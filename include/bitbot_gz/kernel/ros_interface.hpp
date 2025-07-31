#ifndef GZ_NODE_HPP
#define GZ_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace bitbot {

using namespace std::chrono_literals;

class RosInterface : public rclcpp::Node {
 public:
  using Ptr = std::shared_ptr<RosInterface>;
  RosInterface() : rclcpp::Node("bitbot_ros_interface") {
    timer_ready_.store(false);

    joint_command_publisher_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands", 10);
    joint_state_subscriber_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(this->data_lock_);
              this->joint_state_msg_ = msg;
            });
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(this->data_lock_);
          this->imu_msg_ = msg;
        });
    clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10, [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
          this->clock_count_++;
          // Clock rate is 1000
          timer_ready_.store(true);
        });
  }

  ~RosInterface() = default;

  void PublishJointCommand() {
    joint_command_publisher_->publish(joint_command_msg_);
  }

  sensor_msgs::msg::JointState::SharedPtr GetJointState() {
    std::lock_guard<std::mutex> lock(data_lock_);
    return joint_state_msg_;
  }

  std_msgs::msg::Float64MultiArray& GetJointCommand() {
    return joint_command_msg_;
  }

  sensor_msgs::msg::Imu::SharedPtr GetImuData() {
    std::lock_guard<std::mutex> lock(data_lock_);
    return imu_msg_;
  }

  bool IsClockReady() {
    if (timer_ready_.load()) {
      timer_ready_.store(false);
      return true;
    }
    return false;
  }

  bool IsSystemReady() {
    std::lock_guard<std::mutex> lock(this->data_lock_);
    if (joint_state_msg_ != nullptr && imu_msg_ != nullptr)
      return true;
    else
      return false;
  }

  static void RunRosSpin(SharedPtr ptr) {
    std::thread ros_loop([ptr]() { rclcpp::spin(ptr); });
    ros_loop.join();
  }

 private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      joint_command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;

  size_t clock_count_ = 0;

  std::mutex data_lock_;
  sensor_msgs::msg::JointState::SharedPtr joint_state_msg_;
  std_msgs::msg::Float64MultiArray joint_command_msg_;
  sensor_msgs::msg::Imu::SharedPtr imu_msg_;
  std::atomic_bool timer_ready_;
};

}  // namespace bitbot

#endif  // !GZ_NODE_HPP
