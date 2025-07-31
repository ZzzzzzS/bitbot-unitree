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
  virtual void UpdateModel() final;
  virtual void Input() final;
  virtual void Output() final;
  virtual void UpdateRuntimeData() final;
  inline void Rot2RPY() {
    constexpr int row_num = 3;
    roll_ = std::atan2(rot_mat_[2 * row_num + 1], rot_mat_[2 * row_num + 2]);
    pitch_ =
        std::atan2(-rot_mat_[2 * row_num],
                   sqrt(rot_mat_[2 * row_num + 1] * rot_mat_[2 * row_num + 1] +
                        rot_mat_[2 * row_num + 2] * rot_mat_[2 * row_num + 2]));
    yaw_ = std::atan2(rot_mat_[1 * row_num], rot_mat_[0]);
  }

  std::string mj_site_name_;
  int mj_site_id_ = 0;
  int mj_site_xmat_adr_ = 0;
  bool has_acc_ = false;
  std::string mj_acc_name_;
  int mj_acc_id_ = 0;
  int mj_acc_adr_ = 0;
  bool has_gyro_ = false;
  std::string mj_gyro_name_;
  int mj_gyro_id_ = 0;
  int mj_gyro_adr_ = 0;

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
