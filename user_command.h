/**
 * @file user_command.h
 * @author zishun zhou
 * @brief 该文件定义了用户命令的相关数据结构和函数
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#ifdef BUILD_SIMULATION
#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#else
#include "Bitbot_Unitree/include/kernel/unitree_kernel.hpp"
#endif // BUILD_SIMULATION
#include "types.hpp"
#include "user_func.h"


enum Events
{
    EventInitPose = 1001,
    EventPolicyRun,
    EventSystemTest,
    EventPolicyDance,
    EVENT_END
};

enum UserCmdEvents
{
    //注意这里的事件ID的范围不要和系统事件，以及Events中的ID冲突
    VeloxIncrease = 2001,
    VeloxDecrease,
    VeloyIncrease,
    VeloyDecrease,
    VeloYawIncrease,
    VeloYawDecrease,
    // PosZIncrease,
    // PosZDecrease,
    // PosPitchIncrease,
    // PosPitchDecrease,
    // PosRollIncrease,
    // PosRollDecrease,


    GamepadXMove,
    GamepadYMove,
    GamepadYawMove,
    // GamepadZMove,
    // GamepadPitchMove,
    // GamepadRollMove,

    USER_CMD_EVENT_END
};

enum DanceEvnets
{
    EventStartDance = 3001,
    DANCE_EVENT_END
};



std::optional<bitbot::StateId> EventInitPoseFunc(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventPolicyRunFunc(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventPolicyDanceFunc(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventSystemTestFunc(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventStartDanceFunc(bitbot::EventValue value, UserData& user_data);


template<size_t N>
std::optional<bitbot::StateId> EventCmdNIncrease(bitbot::EventValue key_state, UserData& d)
{
    if (key_state == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    { //设置x轴速度
        d.WalkCmdWorker->IncreaseCmd(N);
    }
    return std::optional<bitbot::StateId>();
}

template<size_t N>
std::optional<bitbot::StateId> EventCmdNDecrease(bitbot::EventValue key_state, UserData& d)
{
    if (key_state == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    { //设置x轴速度
        d.WalkCmdWorker->DecreaseCmd(N);
    }
    return std::optional<bitbot::StateId>();
}

template<size_t N>
std::optional<bitbot::StateId> EventCmdNMove(bitbot::EventValue value, UserData& d)
{
    double vel = static_cast<double>(value / 32768.0);
    d.WalkCmdWorker->SetCmd(N, static_cast<RealNumber>(vel));
    return std::optional<bitbot::StateId>();
}