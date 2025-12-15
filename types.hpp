/**
 * @file types.hpp
 * @author Zishun Zhou
 * @brief 类型定义
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <array>

#include "Schedulers/AbstractScheduler.hpp"

#include "Utils/StaticStringUtils.hpp"
#include "Utils/MathTypes.hpp"

#include "Workers/AbstractWorker.hpp"
#include "Workers/AsyncLoggerWorker.hpp"
#include "Workers/ImuProcessWorker.hpp"
#include "Workers/MotorControlWorker.hpp"
#include "Workers/MotorResetPositionWorker.hpp"
#include "Workers/NetCmdWorker.hpp"
#include "Workers/ActionManagementWorker.hpp"
#include "Workers/NN/BeyondMimicWorker.hpp"

#ifdef BUILD_SIMULATION
#include "bitbot_mujoco/device/mujoco_imu.h"
#include "bitbot_mujoco/device/mujoco_joint.h"
using DeviceImu = bitbot::MujocoImu;
using DeviceJoint = bitbot::MujocoJoint;
#else
#include "Bitbot_Unitree/include/device/unitree_imu.h"
#include "Bitbot_Unitree/include/device/unitree_joint.h"
#include "Bitbot_Unitree/include/device/unitree_gamepad.h"
using DeviceImu = bitbot::UnitreeImu;
using DeviceJoint = bitbot::UnitreeJoint;
#endif



/************ basic definintion***********/
using RealNumber = float;
constexpr size_t JOINT_NUMBER = 29;
using Vec3 = z::math::Vector<RealNumber, 3>;
using MotorVec = z::math::Vector<RealNumber, JOINT_NUMBER>;
//constexpr size_t DANCE_TRAJECTORY_LENGTH = 1749; //NOTE: remember to change this when changing dancing trajectories dance 102
constexpr size_t DANCE_TRAJECTORY_LENGTH = 1942; //NOTE: remember to change this when changing dancing trajectories gannam style
//constexpr size_t DANCE_TRAJECTORY_LENGTH = 2897; //NOTE: remember to change this when changing dancing trajectories kuailechongbai

#ifdef BUILD_SIMULATION
constexpr std::array<size_t, JOINT_NUMBER> JOINT_ID_MAP = {
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28
};
constexpr size_t IMU_ID_MAP = 29;
constexpr size_t ALTER_IMU_ID_MAP = 30;
#else
constexpr std::array<size_t, JOINT_NUMBER> JOINT_ID_MAP = {
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28
}; // Unitree joint mapping is the same as simulation
constexpr size_t IMU_ID_MAP = 30; // Unitree IMU ID
constexpr size_t ALTER_IMU_ID_MAP = 31; //unitree alter IMU ID
#endif

/********** IMU Data Pair******************/
constexpr z::CTSPair<"AccelerationRaw", Vec3> ImuAccRawPair;
constexpr z::CTSPair<"AngleVelocityRaw", Vec3> ImuGyroRawPair;
constexpr z::CTSPair<"AngleRaw", Vec3> ImuMagRawPair;

constexpr z::CTSPair<"AccelerationValue", Vec3> ImuAccFilteredPair;
constexpr z::CTSPair<"AngleValue", Vec3> ImuMagFilteredPair;
constexpr z::CTSPair<"AngleVelocityValue", Vec3> ImuGyroFilteredPair;
constexpr z::CTSPair<"AlterAngleValue", Vec3> ImuAlterAngleFilteredPair;


/********** Motor control Pair ************/
constexpr z::CTSPair<"TargetMotorPosition", MotorVec> TargetMotorPosPair;
constexpr z::CTSPair<"TargetMotorVelocity", MotorVec> TargetMotorVelPair;
constexpr z::CTSPair<"TargetMotorTorque", MotorVec> TargetMotorTorquePair;
constexpr z::CTSPair<"CurrentMotorPosition", MotorVec> CurrentMotorPosPair;
constexpr z::CTSPair<"CurrentMotorVelocity", MotorVec> CurrentMotorVelPair;
constexpr z::CTSPair<"CurrentMotorTorque", MotorVec> CurrentMotorTorquePair;
constexpr z::CTSPair<"LimitTargetMotorTorque", MotorVec> LimitTargetMotorTorquePair;
constexpr z::CTSPair<"CurrentMotorPositionRaw", MotorVec> CurrentMotorPosRawPair;
constexpr z::CTSPair<"CurrentMotorVelocityRaw", MotorVec> CurrentMotorVelRawPair;

/********* NN pair ********************/
constexpr z::CTString Net1Name = "Net1";
constexpr z::CTSPair<z::concat(Net1Name, "NetLastAction"), MotorVec> NetLastActionPair;
constexpr z::CTSPair<z::concat(Net1Name, "Action"), MotorVec> Net1OutPair;
constexpr z::CTSPair<z::concat(Net1Name, "InferenceTime"), RealNumber> InferenceTimePair;
constexpr z::CTSPair<z::concat(Net1Name, "RefTraj"), MotorVec> Net1RefTrajPair;
constexpr z::CTSPair<z::concat(Net1Name, "RefVel"), MotorVec> Net1RefVelPair;


// define scheduler
using SchedulerType = z::AbstractScheduler<ImuAccRawPair, ImuGyroRawPair, ImuMagRawPair,
    ImuAccFilteredPair, ImuGyroFilteredPair, ImuMagFilteredPair,
    TargetMotorPosPair, TargetMotorVelPair, CurrentMotorPosPair, CurrentMotorVelPair, CurrentMotorTorquePair,
    TargetMotorTorquePair, LimitTargetMotorTorquePair,
    CurrentMotorVelRawPair, CurrentMotorPosRawPair,
    NetLastActionPair, InferenceTimePair, Net1OutPair, Net1RefTrajPair, Net1RefVelPair, ImuAlterAngleFilteredPair>;


//define workers
using MotorResetWorkerType = z::MotorResetPositionWorker<SchedulerType, RealNumber, JOINT_NUMBER>;
using ImuWorkerType = z::ImuProcessWorker<SchedulerType, DeviceImu*, RealNumber>;
using AlterImuWorkerType = z::SimpleCallbackWorker<SchedulerType>;
using MotorWorkerType = z::MotorControlWorker<SchedulerType, DeviceJoint*, RealNumber, JOINT_NUMBER>;
using LoggerWorkerType = z::AsyncLoggerWorker<SchedulerType, RealNumber, ImuAccRawPair, ImuGyroRawPair, ImuMagRawPair,
    ImuAccFilteredPair, ImuGyroFilteredPair, ImuMagFilteredPair,
    TargetMotorPosPair, TargetMotorVelPair, CurrentMotorPosPair, CurrentMotorVelPair, CurrentMotorTorquePair,
    TargetMotorTorquePair, LimitTargetMotorTorquePair,
    NetLastActionPair, InferenceTimePair, Net1OutPair, Net1RefTrajPair, Net1RefVelPair, ImuAlterAngleFilteredPair>;

using ActionManagementWorkerType = z::ActionManagementWorker<SchedulerType, RealNumber, Net1OutPair>;

/******define actor net************/
using BeyondMimicUnitreeInferWorkerType = z::BeyondMimicUnitreeInferenceWorker<SchedulerType, Net1Name, RealNumber, JOINT_NUMBER, DANCE_TRAJECTORY_LENGTH>;
