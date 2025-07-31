#include "bitbot_gz/device/gz_joint.h"

namespace bitbot {

GzJoint::GzJoint(const pugi::xml_node& device_node) : GzDevice(device_node) {
  basic_type_ = (uint32_t)BasicDeviceType::MOTOR;
  type_ = (uint32_t)GzDeviceType::GZ_JOINT;

  monitor_header_.headers = {"mode",
                             "actual_position",
                             "target_position",
                             "actual_velocity",
                             "target_velocity",
                             "actual_torque",
                             "target_torque"};
  monitor_data_.resize(monitor_header_.headers.size());

  target_position_ = 0.0;

  ConfigParser::ParseAttribute2d(initial_pos_,
                                 device_node.attribute("initial_pos"));
  ConfigParser::ParseAttribute2b(enable_, device_node.attribute("enable"));

  std::string mode_str;
  ConfigParser::ParseAttribute2s(mode_str, device_node.attribute("mode"));
  if (mode_str == "position") {
    joint_type_ = GzJointType::POSITION;
  } else if (mode_str == "velocity") {
    joint_type_ = GzJointType::VELOCITY;
  } else if (mode_str == "torque") {
    joint_type_ = GzJointType::TORQUE;
  } else {
    joint_type_ = GzJointType::NONE;
  }
  ConfigParser::ParseAttribute2d(p_gain_, device_node.attribute("p_gain"));
  ConfigParser::ParseAttribute2d(d_gain_, device_node.attribute("d_gain"));
  ConfigParser::ParseAttribute2d(i_gain_, device_node.attribute("i_gain"));
}

GzJoint::~GzJoint() {}

void GzJoint::Input(const RosInterface::Ptr ros_interface) {
  auto joint_state_msg = ros_interface->GetJointState();
  // TODO
}

void GzJoint::Output(const RosInterface::Ptr ros_interface) {
  auto& joint_command_msg = ros_interface->GetJointCommand();
  // TODO
}

void GzJoint::UpdateRuntimeData() {
  constexpr double rad2deg = 180.0 / M_PI;

  monitor_data_[0] = (int)joint_type_;
  monitor_data_[1] = rad2deg * actual_position_;
  monitor_data_[2] = rad2deg * target_position_;
  monitor_data_[3] = actual_velocity_;
  monitor_data_[4] = target_velocity_;
  monitor_data_[5] = actual_torque_;
  monitor_data_[6] = target_torque_;
}

}  // namespace bitbot
