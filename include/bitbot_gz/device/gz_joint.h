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
  virtual void Input(const RosInterface::Ptr ros_interface) final;
  virtual void Output(const RosInterface::Ptr ros_interface) final;
  virtual void UpdateRuntimeData() final;

  inline double GetActualPosition() { return actual_position_; }

  inline double GetActualVelocity() { return actual_velocity_; }

  inline double GetActualTorque() { return actual_torque_; }

  inline void SetTargetPosition(double pos) { target_position_ = pos; }

  inline void SetTargetVelocity(double vel) { target_velocity_ = vel; }

  inline void SetTargetTorque(double torque) { target_torque_ = torque; }

 private:
  GzJointType joint_type_;
  bool enable_ = true;

  double initial_pos_ = 0.0;
  double p_gain_ = 0.0;
  double d_gain_ = 0.0;
  double i_gain_ = 0.0;

  double actual_position_ = 0.0;
  double actual_velocity_ = 0.0;
  double actual_torque_ = 0.0;
  double target_position_ = 0.0;
  double target_velocity_ = 0.0;
  double target_torque_ = 0.0;
};

}  // namespace bitbot

#endif  // !GZ_JOINT_H
