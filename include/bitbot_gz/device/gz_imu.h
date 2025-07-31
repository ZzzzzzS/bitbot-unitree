#ifndef GZ_IMU_H
#define GZ_IMU_H

#include "bitbot_gz/device/gz_device.hpp"

namespace bitbot {

class GzImu final : public GzDevice {
 public:
  GzImu(pugi::xml_node const& device_node);
  ~GzImu();

  inline double GetRoll() { return roll_; }

  inline double GetPitch() { return pitch_; }

  inline double GetYaw() { return yaw_; }

  inline std::array<double, 9> GetRotationMatrix() { return rot_mat_; }

  inline double GetAccX() { return acc_x_; }

  inline double GetAccY() { return acc_y_; }

  inline double GetAccZ() { return acc_z_; }

  inline double GetGyroX() { return gyro_x_; }

  inline double GetGyroY() { return gyro_y_; }

  inline double GetGyroZ() { return gyro_z_; }

 private:
  virtual void Input(const RosInterface::Ptr ros_interface) final;
  virtual void Output(const RosInterface::Ptr ros_interface) final;
  virtual void UpdateRuntimeData() final;

  double roll_ = 0;
  double pitch_ = 0;
  double yaw_ = 0;
  std::array<double, 9> rot_mat_ = {0};
  double acc_x_ = 0;
  double acc_y_ = 0;
  double acc_z_ = 0;
  double gyro_x_ = 0;
  double gyro_y_ = 0;
  double gyro_z_ = 0;
};

}  // namespace bitbot

#endif  // !GZ_IMU_H
