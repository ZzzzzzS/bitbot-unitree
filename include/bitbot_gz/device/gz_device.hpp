#ifndef GZ_DEVICE_HPP
#define GZ_DEVICE_HPP

#include "bitbot_gz/kernel/ros_interface.hpp"
#include "bitbot_kernel/device/device.hpp"

namespace bitbot {

enum class GzDeviceType : uint32_t {
  GZ_DEVICE = 12000,
  GZ_JOINT,
  GZ_FORCE_SENSOR,
  GZ_IMU,
  GZ_POSITION,
  GZ_LINVEL,
};

class GzDevice : public Device {
 public:
  GzDevice(const pugi::xml_node& device_node) : Device(device_node) {}
  ~GzDevice() = default;

  // Method to w/r gz simulation
  // virtual void UpdateModel() = 0;
  virtual void Input(const RosInterface::Ptr ros_interface) = 0;
  virtual void Output(const RosInterface::Ptr ros_interface) = 0;

  // Inherited from Device base calss
  virtual void UpdateRuntimeData() = 0;

 private:
};
}  // namespace bitbot

#endif  // !GZ_DEVICE_HPP
