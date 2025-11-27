#ifndef GZ_IMU_H
#define GZ_IMU_H

#include "device/gz_device.hpp"

namespace bitbot {

  class GzImu final : public GzDevice {
  public:
    GzImu(pugi::xml_node const& device_node);
    ~GzImu();

    inline float GetRoll() { return roll_; }
    inline float GetPitch() { return pitch_; }
    inline float GetYaw() { return yaw_; }
    inline float GetAccX() { return acc_x_; }
    inline float GetAccY() { return acc_y_; }
    inline float GetAccZ() { return acc_z_; }
    inline float GetGyroX() { return gyro_x_; }
    inline float GetGyroY() { return gyro_y_; }
    inline float GetGyroZ() { return gyro_z_; }

  private:
    virtual void Input(const IOType& IO) final;
    virtual IOType Output() final;
    virtual void UpdateRuntimeData() final;

    float roll_ = 0;
    float pitch_ = 0;
    float yaw_ = 0;
    float acc_x_ = 0;
    float acc_y_ = 0;
    float acc_z_ = 0;
    float gyro_x_ = 0;
    float gyro_y_ = 0;
    float gyro_z_ = 0;
  };

}  // namespace bitbot

#endif  // !GZ_IMU_H
