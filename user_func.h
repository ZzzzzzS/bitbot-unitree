#pragma once
#ifdef BUILD_SIMULATION
#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#else
#include "Bitbot_Unitree/include/kernel/gz_kernel.hpp"
#endif // BUILD_SIMULATION

#ifdef BUILD_SIMULATION
#include "bitbot_mujoco/device/mujoco_imu.h"
#include "bitbot_mujoco/device/mujoco_joint.h"
#else
#include "Bitbot_Unitree/include/device/gz_joint.h"
#include "Bitbot_Unitree/include/device/gz_imu.h"
#endif //BUILD_SIMULATION

constexpr size_t JOINT_NUMBER = 6;
#ifdef BUILD_SIMULATION
using DeviceImu = bitbot::MujocoImu;
using DeviceJoint = bitbot::MujocoJoint;
constexpr std::array<size_t, JOINT_NUMBER> JOINT_ID_MAP = { 0,1,2,3,4,5 };
constexpr size_t IMU_ID_MAP = 6;
#else
using DeviceImu = bitbot::GzImu;
using DeviceJoint = bitbot::GzJoint;
constexpr std::array<size_t, JOINT_NUMBER> JOINT_ID_MAP = { 0,1,2,3,4,5 };
constexpr size_t IMU_ID_MAP = 30;
#endif //BUILD_SIMULATION


enum Events
{
    InitPose = 1001,
    PolicyRun,
    SystemTest,

    VeloxIncrease = 2001,
    VeloxDecrease = 2002,
    VeloyIncrease = 2003,
    VeloyDecrease = 2004,

    GamepadInitPose = 3002,
    GamepadPolicyRun = 3003,
    GamepadVeloxIncreaseDisc = 3101,
    GamepadVeloxDecreaseDisc = 3102,
    GamepadVeloyIncreaseDisc = 3103,
    GamepadVeloyDecreaseDisc = 3104
};

enum class States : bitbot::StateId
{
    Waiting = 1001,
    PF2InitPose,
    PF2PolicyRun,
    PF2SystemTest,
};



struct UserData
{
    std::array<DeviceJoint*, JOINT_NUMBER> JointsPtr;
    DeviceImu* ImuPtr;
};

#ifdef BUILD_SIMULATION
using KernelType = bitbot::MujocoKernel<UserData>;
using KernelBus = bitbot::MujocoBus;
#else
using KernelType = bitbot::GzKernel<UserData>;
using KernelBus = bitbot::GzBus;
#endif //BUILD_SIMULATION


std::optional<bitbot::StateId> EventInitPose(bitbot::EventValue value,
    UserData& user_data);
std::optional<bitbot::StateId> EventPolicyRun(bitbot::EventValue value,
    UserData& user_data);
std::optional<bitbot::StateId> EventFakePowerOn(bitbot::EventValue value,
    UserData& user_data);
std::optional<bitbot::StateId> EventSystemTest(bitbot::EventValue value,
    UserData& user_data);

std::optional<bitbot::StateId> EventVeloXIncrease(bitbot::EventValue keyState, UserData& d);
std::optional<bitbot::StateId> EventVeloXDecrease(bitbot::EventValue keyState, UserData& d);
std::optional<bitbot::StateId> EventVeloYIncrease(bitbot::EventValue keyState, UserData& d);
std::optional<bitbot::StateId> EventVeloYDecrease(bitbot::EventValue keyState, UserData& d);


void ConfigFunc(const KernelBus& bus, UserData& d);
void FinishFunc(UserData& d);

void StateWaiting(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& user_data);

void StateJointInitPose(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& user_data);

void StatePolicyRun(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& user_data);


void StateSystemTest(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& user_data);
