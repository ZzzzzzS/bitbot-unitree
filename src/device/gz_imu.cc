#include "bitbot_gz/device/gz_imu.h"

namespace bitbot {

GzImu::GzImu(pugi::xml_node const& device_node) : GzDevice(device_node) {
  basic_type_ = (uint32_t)BasicDeviceType::IMU;
  type_ = (uint32_t)GzDeviceType::GZ_IMU;

  monitor_header_.headers = {"roll",  "pitch",  "yaw",    "acc_x", "acc_y",
                             "acc_z", "gyro_x", "gyro_y", "gyro_z"};
  monitor_data_.resize(monitor_header_.headers.size());
}

GzImu::~GzImu() {}

void GzImu::Input(const RosInterface::Ptr ros_interface) {
  auto imu_msg = ros_interface->GetImuData();
}

void GzImu::Output(const RosInterface::Ptr) {}

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
