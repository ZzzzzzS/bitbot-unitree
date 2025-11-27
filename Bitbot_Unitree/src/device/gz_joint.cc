#include "device/gz_joint.h"

namespace bitbot {

  GzJoint::GzJoint(const pugi::xml_node& device_node) : GzDevice(device_node) {
    basic_type_ = (uint32_t)BasicDeviceType::MOTOR;
    type_ = (uint32_t)GzDeviceType::GZ_JOINT;

    monitor_header_.headers = { "mode",
                               "actual_position",
                               "target_position",
                               "actual_velocity",
                               "target_velocity",
                               "actual_torque",
                               "target_torque",
                               "temperature1",
                               "temperature2" };
    monitor_data_.resize(monitor_header_.headers.size());
    ConfigParser::ParseAttribute2b(enable_, device_node.attribute("enable"));

    double p_gain, d_gain;
    ConfigParser::ParseAttribute2d(p_gain, device_node.attribute("kp"));
    ConfigParser::ParseAttribute2d(d_gain, device_node.attribute("kd"));
    this->p_gain_ = static_cast<float>(p_gain);
    this->d_gain_ = static_cast<float>(d_gain);
  }

  GzJoint::~GzJoint() {}

  void GzJoint::Input(const IOType& IO) {
    auto motor_state = std::get<unitree_hg::msg::dds_::MotorState_>(IO);
    this->actual_position_ = motor_state.q();
    this->actual_velocity_ = motor_state.dq();
    this->actual_torque_ = motor_state.tau_est();
    this->mode_ = motor_state.mode();
    this->temp[0] = motor_state.temperature()[0];
    this->temp[1] = motor_state.temperature()[1];
  }

  void GzJoint::PowerOn()
  {
    this->target_position_ = this->actual_position_;
    this->target_velocity_ = 0;
    this->target_torque_ = 0;
    this->power_on_ = true;
  }

  void GzJoint::PowerOff()
  {
    power_on_ = false;
    this->target_position_ = this->actual_position_;
    this->target_velocity_ = 0;
    this->target_torque_ = 0;
  }

  IOType GzJoint::Output() {
    unitree_hg::msg::dds_::MotorCmd_ motor_cmd;
    if (power_on_ && enable_) { //必须上电且使能才能控制
      motor_cmd.mode() = 1;
      motor_cmd.q() = target_position_;
      motor_cmd.dq() = target_velocity_;
      motor_cmd.tau() = target_torque_;
      motor_cmd.kp() = p_gain_;
      motor_cmd.kd() = d_gain_;
    }
    else {
      motor_cmd.mode() = 0;  // Idle
      motor_cmd.q() = 0.0;
      motor_cmd.dq() = 0.0;
      motor_cmd.tau() = 0.0;
      motor_cmd.kp() = 0.0;
      motor_cmd.kd() = 0.0;
    }
    return motor_cmd;
  }

  void GzJoint::UpdateRuntimeData() {
    constexpr double rad2deg = 180.0 / M_PI;

    monitor_data_[0] = mode_;
    monitor_data_[1] = rad2deg * actual_position_;
    monitor_data_[2] = rad2deg * target_position_;
    monitor_data_[3] = actual_velocity_;
    monitor_data_[4] = target_velocity_;
    monitor_data_[5] = actual_torque_;
    monitor_data_[6] = target_torque_;
    monitor_data_[7] = static_cast<float>(temp[0]);
    monitor_data_[8] = static_cast<float>(temp[1]);
  }


}  // namespace bitbot
