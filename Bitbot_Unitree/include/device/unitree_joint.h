#pragma once

#include "device/unitree_device.hpp"

namespace bitbot {

  class UnitreeJoint final : public UnitreeDevice {
  public:
    UnitreeJoint(const pugi::xml_node& device_node);
    ~UnitreeJoint();

    inline float GetActualPosition() { return actual_position_; }
    inline float GetActualVelocity() { return actual_velocity_; }
    inline float GetActualTorque() { return actual_torque_; }
    inline void SetTargetPosition(float pos) { target_position_ = pos; }
    inline void SetTargetVelocity(float vel) { target_velocity_ = vel; }
    inline void SetTargetTorque(float torque) { target_torque_ = torque; }
    inline void SetMotionKp(float p) { p_gain_ = p; }
    inline void SetMotionKd(float d) { d_gain_ = d; }
    inline float GetMotionKp() { return p_gain_; }
    inline float GetMotionKd() { return d_gain_; }

    void PowerOn();
    void PowerOff();

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

