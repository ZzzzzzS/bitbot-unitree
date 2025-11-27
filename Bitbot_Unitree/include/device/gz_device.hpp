#ifndef GZ_DEVICE_HPP
#define GZ_DEVICE_HPP

#include "variant"
#include "bitbot_kernel/device/device.hpp"

#include "unitree/idl/hg/MotorState_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/hg/IMUState_.hpp"
#include "unitree/idl/hg/MainBoardState_.hpp"
#include "unitree/idl/hg/BmsState_.hpp"
#include "unitree/idl/hg/MotorCmd_.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"



namespace bitbot {

  enum class GzDeviceType : uint32_t {
    GZ_DEVICE = 1000,
    GZ_JOINT,
    GZ_IMU,
    GZ_JOYSTICK,
    GZ_BATTERY,
    GZ_MOTHERBOARD
  };


  using IOType = std::variant<unitree_hg::msg::dds_::MotorState_,
    unitree_hg::msg::dds_::IMUState_,
    unitree_hg::msg::dds_::MainBoardState_,
    unitree_hg::msg::dds_::BmsState_,
    unitree_hg::msg::dds_::MotorCmd_>;

  class GzDevice : public Device {
  public:
    GzDevice(const pugi::xml_node& device_node) : Device(device_node) {}
    ~GzDevice() = default;

    // Method to w/r gz simulation
    virtual void Input(const IOType& IO) = 0;
    virtual IOType Output() = 0;


    // Inherited from Device base calss
    virtual void UpdateRuntimeData() = 0;

  private:
  };
}  // namespace bitbot

#endif  // !GZ_DEVICE_HPP
