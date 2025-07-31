#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class TorquePublisher : public rclcpp::Node {
 public:
  TorquePublisher() : Node("torque_publisher") {
    // Publisher for torque commands
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", 10);

    // Subscribers for sensor data
    joint_state_subscriber_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TorquePublisher::joint_state_callback, this,
                      std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_data", 10,
        std::bind(&TorquePublisher::imu_callback, this, std::placeholders::_1));

    // Timer for publishing torque commands
    timer_ = this->create_wall_timer(
        100ms, std::bind(&TorquePublisher::timer_callback, this));

    // Initialize variables
    joint1_pos_ = 0.0;
    joint1_vel_ = 0.0;
    joint2_pos_ = 0.0;
    joint2_vel_ = 0.0;
    imu_received_ = false;
  }

 private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Find joint1 and joint2 indices
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "joint1") {
        joint1_pos_ = msg->position[i];
        joint1_vel_ = msg->velocity[i];
      } else if (msg->name[i] == "joint2") {
        joint2_pos_ = msg->position[i];
        joint2_vel_ = msg->velocity[i];
      }
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_data_ = *msg;
    imu_received_ = true;
  }

  void timer_callback() {
    auto msg = std_msgs::msg::Float64MultiArray();

    // Set torque commands for joint1 and joint2
    msg.data = {5.0, -2.0};  // Torque values in Nm

    publisher_->publish(msg);

    // Print all sensor data
    RCLCPP_INFO(this->get_logger(), "=== ROBOT STATE ===");
    RCLCPP_INFO(this->get_logger(),
                "Joint1: pos=%.3f rad (%.1f°), vel=%.3f rad/s", joint1_pos_,
                joint1_pos_ * 180.0 / M_PI, joint1_vel_);
    RCLCPP_INFO(this->get_logger(),
                "Joint2: pos=%.3f rad (%.1f°), vel=%.3f rad/s", joint2_pos_,
                joint2_pos_ * 180.0 / M_PI, joint2_vel_);
    RCLCPP_INFO(this->get_logger(), "Torque commands: [%.2f, %.2f] Nm",
                msg.data[0], msg.data[1]);

    if (imu_received_) {
      RCLCPP_INFO(
          this->get_logger(), "IMU - Linear accel: x=%.3f, y=%.3f, z=%.3f m/s²",
          imu_data_.linear_acceleration.x, imu_data_.linear_acceleration.y,
          imu_data_.linear_acceleration.z);
      RCLCPP_INFO(this->get_logger(),
                  "IMU - Angular vel: x=%.3f, y=%.3f, z=%.3f rad/s",
                  imu_data_.angular_velocity.x, imu_data_.angular_velocity.y,
                  imu_data_.angular_velocity.z);
      RCLCPP_INFO(this->get_logger(),
                  "IMU - Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                  imu_data_.orientation.x, imu_data_.orientation.y,
                  imu_data_.orientation.z, imu_data_.orientation.w);
    } else {
      RCLCPP_INFO(this->get_logger(), "IMU - No data received yet");
    }
    RCLCPP_INFO(this->get_logger(), "==================");
  }

 private:
  // Publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Joint state variables
  double joint1_pos_, joint1_vel_;
  double joint2_pos_, joint2_vel_;

  // IMU data
  sensor_msgs::msg::Imu imu_data_;
  bool imu_received_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorquePublisher>());
  rclcpp::shutdown();
  return 0;
}
