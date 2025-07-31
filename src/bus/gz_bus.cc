#include "bitbot_gz/bus/gz_bus.h"

namespace bitbot {

GzBus::GzBus() {}

GzBus::~GzBus() {}

void GzBus::doConfigure(const pugi::xml_node& bus_node) {
  CreateDevices(bus_node);
}

void GzBus::doRegisterDevices() {
  static DeviceRegistrar<GzDevice, GzJoint> gz_joint(
      (uint32_t)GzDeviceType::GZ_JOINT, "GzJoint");
  static DeviceRegistrar<GzDevice, GzImu> gz_imu((uint32_t)GzDeviceType::GZ_IMU,
                                                 "GzImu");
}

void GzBus::WriteBus() {
  for (auto& device : devices_) {
    device->Output();
  }
}

void GzBus::ReadBus() {
  for (auto& device : devices_) {
    device->Input();
  }
}

void GzBus::UpdateDevices() {
  for (auto& device : devices_) {
    device->UpdateModel();
  }
}

}  // namespace bitbot
