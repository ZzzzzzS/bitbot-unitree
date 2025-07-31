#ifndef GZ_JOINT_H
#define GZ_JOINT_H

#include "bitbot_gz/device/gz_device.hpp"

namespace bitbot {

enum class GzJointType {
  NONE = 0,
  POSITION,
  VELOCITY,
  TORQUE,
};

class GzJoint final : public GzDevice {
 public:
  GzJoint(const pugi::xml_node& device_node);
  ~GzJoint();

 private:
  virtual void UpdateModel() final;
  virtual void Input() final;
  virtual void Output() final;
  virtual void UpdateRuntimeData() final;

 private:
  GzJointType joint_type_;
  bool enable_ = true;
  double actual_position_ = 0;
  double actual_velocity_ = 0;
  double actual_torque_ = 0;
  double target_position_ = 0;
  double target_velocity_ = 0;
  double target_torque_ = 0;
  double p_gain = 0.0;
  double d_gain = 0.0;
  double i_gain = 0.0;
};

}  // namespace bitbot

#endif  // !GZ_JOINT_H
