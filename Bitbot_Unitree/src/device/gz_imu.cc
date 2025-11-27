#include "device/gz_imu.h"

namespace bitbot {

  GzImu::GzImu(pugi::xml_node const& device_node) : GzDevice(device_node) {
    basic_type_ = (uint32_t)BasicDeviceType::IMU;
    type_ = (uint32_t)GzDeviceType::GZ_IMU;
    monitor_header_.headers = { "roll",  "pitch",  "yaw", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z" };
    monitor_data_.resize(monitor_header_.headers.size());
  }

  GzImu::~GzImu() {}

  void GzImu::Input(const IOType& IO) {
    auto imu_state = std::get<unitree_hg::msg::dds_::IMUState_>(IO);
    this->roll_ = imu_state.rpy()[0];
    this->pitch_ = imu_state.rpy()[1];
    this->yaw_ = imu_state.rpy()[2];
    this->gyro_x_ = imu_state.gyroscope()[0];
    this->gyro_y_ = imu_state.gyroscope()[1];
    this->gyro_z_ = imu_state.gyroscope()[2];
    this->acc_x_ = imu_state.accelerometer()[0];
    this->acc_y_ = imu_state.accelerometer()[1];
    this->acc_z_ = imu_state.accelerometer()[2];
  }

  IOType GzImu::Output() {
    return IOType();
  }

  void GzImu::UpdateRuntimeData() {
    constexpr double rad2deg = 180.0 / M_PI;

    monitor_data_[0] = rad2deg * roll_;
    monitor_data_[1] = rad2deg * pitch_;
    monitor_data_[2] = rad2deg * yaw_;
    monitor_data_[3] = acc_x_;
    monitor_data_[4] = acc_y_;
    monitor_data_[5] = acc_z_;
    monitor_data_[6] = gyro_x_;
    monitor_data_[7] = gyro_y_;
    monitor_data_[8] = gyro_z_;
  }

}  // namespace bitbot
