#ifndef GZ_BUS_H
#define GZ_BUS_H

#include "device/gz_device.hpp"
#include "device/gz_imu.h"
#include "device/gz_joint.h"
#include "bitbot_kernel/bus/bus_manager.hpp"

#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

#include "thread"
#include "mutex"

#include "pugixml.hpp"

namespace bitbot {
  class GzBus : public BusManagerTpl<GzBus, GzDevice> {
  public:
    GzBus();
    ~GzBus();

    void WriteBus();
    void ReadBus();
    void RegisterDevices();
    void Init(pugi::xml_node& bitbot_node);
    void PowerOn();
    void PowerOff();
    bool isSystemReady() { return received.load(); }

  private:
    void LowStateCallback(const void* msg);
    void AlterImuCallback(const void* msg);
    void MainBoardStateCallback(const void* msg);
    void BmsStateCallback(const void* msg);

    void InitPublishersAndSubscribers();

  private:
    const std::string LOW_STATE_TOPIC = "rt/lowstate";
    const std::string ALTER_IMU_STATE_TOPIC = "rt/secondary_imu";
    const std::string MAINBOARD_STATE_TOPIC = "rt/lf/mainboardstate";
    const std::string BMS_STATE_TOPIC = "rt/lf/bmsstate";
    const std::string LOW_CMD_TOPIC = "rt/lowcmd";


    //handler 抓手，支撑点，着力点，立足点，发力点，依托点，牛鼻子，总开关。方法，工具，载体，平台。
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> low_command_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::IMUState_> alter_imu_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::MainBoardState_> main_board_state_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::BmsState_> bms_state_subscriber_;

    //注意读写时加锁解锁
    std::mutex motor_state_lock_;
    std::array<unitree_hg::msg::dds_::MotorState_, 35> motor_states_;
    std::mutex motor_cmd_lock_;
    std::array<unitree_hg::msg::dds_::MotorCmd_, 35> motor_cmds_;
    std::mutex imu_state_lock_;
    std::array<unitree_hg::msg::dds_::IMUState_, 2> imu_states_;
    std::mutex main_board_state_lock_;
    unitree_hg::msg::dds_::MainBoardState_ main_board_state_msg_;
    std::mutex bms_state_lock_;
    unitree_hg::msg::dds_::BmsState_ bms_state_msg_;

    // ros bus node spin thread
    std::atomic<bool> received = false;

    //device list
    std::vector<GzDevice*> joint_devices_;
    std::vector<GzDevice*> imu_devices_;
    // GzDevice motherboard_device_;
    // GzDevice battery_device_;
    //TODO: add motherboard and battery device

  private: //LowCmd config
    uint8_t mode_pr_;
    uint8_t mode_machine_;

  };
}  // namespace bitbot

#endif  // !GZ_BUS_H
