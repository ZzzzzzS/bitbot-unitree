#ifndef GZ_BUS_H
#define GZ_BUS_H

#include "bitbot_gz/device/gz_device.hpp"
#include "bitbot_gz/device/gz_imu.h"
#include "bitbot_gz/device/gz_joint.h"
#include "bitbot_gz/kernel/ros_interface.hpp"
#include "bitbot_kernel/bus/bus_manager.hpp"

namespace bitbot {
class GzBus : public BusManagerTpl<GzBus, GzDevice> {
 public:
  GzBus();
  ~GzBus();

  void WriteBus();
  void ReadBus();
  void UpdateDevices();

  inline void SetInterface(const RosInterface::Ptr ros_interface) {
    ros_interface_ = ros_interface;
  }

 protected:
  void doConfigure(const pugi::xml_node& bus_node);
  void doRegisterDevices();

 private:
  RosInterface::Ptr ros_interface_;
};
}  // namespace bitbot

#endif  // !GZ_BUS_H
