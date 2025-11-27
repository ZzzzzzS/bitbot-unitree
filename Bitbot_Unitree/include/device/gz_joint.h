#ifndef GZ_JOINT_H
#define GZ_JOINT_H

#include "device/gz_device.hpp"

namespace bitbot {

  class GzJoint final : public GzDevice {
  public:
    GzJoint(const pugi::xml_node& device_node);
    ~GzJoint();

    inline float GetActualPosition() { return actual_position_; }
    inline float GetActualVelocity() { return actual_velocity_; }
    inline float GetActualTorque() { return actual_torque_; }
    inline void SetTargetPosition(float pos) { target_position_ = pos; }
    inline void SetTargetVelocity(float vel) { target_velocity_ = vel; }
    inline void SetTargetTorque(float torque) { target_torque_ = torque; }

    void PowerOn() { power_on_ = true; }
    void PowerOff() { power_on_ = false; }

  private:
    virtual void Input(const IOType& IO) final;
    virtual IOType Output() final;
    virtual void UpdateRuntimeData() final;

  private:
    bool enable_ = true;
    bool power_on_ = false;

    float p_gain_ = 0.0;
    float d_gain_ = 0.0;
    uint8_t mode_ = 0;
    uint32_t temp[2] = { 0 };

    float actual_position_ = 0.0;
    float actual_velocity_ = 0.0;
    float actual_torque_ = 0.0;
    float target_position_ = 0.0;
    float target_velocity_ = 0.0;
    float target_torque_ = 0.0;
  };

}  // namespace bitbot

#endif  // !GZ_JOINT_H
