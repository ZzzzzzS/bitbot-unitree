/**
 * @file user_func.cpp
 * @author Zishun Zhou
 * @brief
 * @date 2025-03-10
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "user_func.h"

#include <chrono>
#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>
#include <memory>
#include <thread>
#include <iostream> // std::cout
#include <nlohmann/json.hpp>
#include <fstream>
#include "types.hpp"

 // 辅助函数：对容器中的每个元素应用函数
template <typename Container, typename Func>
void Apply(Container& container, Func func)
{
    for (size_t i = 0; i < container.size(); ++i)
    {
        func(&container[i], i);
    }
}

void ConfigFunc(const KernelBus& bus, UserData& d)
{
    //读取json配置文件,并初始化各个worker
    nlohmann::json cfg_root;
    nlohmann::json cfg_workers;
    {
        //NOTE: 注意将配置文件路径修改为自己的路径
        std::string path = PROJECT_ROOT_DIR + std::string("/settings/CtrlConfig.json");
        std::ifstream cfg_file(path);
        cfg_root = nlohmann::json::parse(cfg_file, nullptr, true, true);
        cfg_workers = cfg_root["Workers"];
    }

    //获取设备指针
    d.ImuPtr = bus.GetDevice<DeviceImu>(IMU_ID_MAP).value();
    d.ImuAlterPtr = bus.GetDevice<DeviceImu>(ALTER_IMU_ID_MAP).value();
    Apply(d.JointsPtr, [&bus](DeviceJoint** joint, size_t i)
        { *joint = bus.GetDevice<DeviceJoint>(JOINT_ID_MAP[i]).value(); });

    //创建调度器
    d.TaskScheduler = SchedulerType::Create(cfg_root["Scheduler"]);

    //初始化各个worker
    d.ImuWorker = d.TaskScheduler->template CreateWorker<ImuWorkerType>(d.ImuPtr, cfg_workers["ImuProcess"]);
    d.MotorWorker = d.TaskScheduler->template CreateWorker<MotorWorkerType>(cfg_workers["MotorControl"], d.JointsPtr);
    d.Logger = d.TaskScheduler->template CreateWorker<LoggerWorkerType>(cfg_workers["AsyncLogger"]);
    d.ActionManagementWorker = d.TaskScheduler->template CreateWorker<ActionManagementWorkerType>(cfg_workers["ActionManager"]);
    d.AlterImuWorker = d.TaskScheduler->template CreateWorker<AlterImuWorkerType>([&d](SchedulerType::Ptr scheduler) {
        RealNumber roll = d.ImuAlterPtr->GetRoll();
        RealNumber pitch = d.ImuAlterPtr->GetPitch();
        RealNumber yaw = d.ImuAlterPtr->GetYaw();
        Vec3 euler_angles = Vec3({ roll, pitch, yaw });
        scheduler->template SetData<"AlterAngleValue">(euler_angles);
        });

    //创建主任务列表，并添加worker
    d.TaskScheduler->CreateTaskList("MainTask", 1, true);
    d.TaskScheduler->AddWorkers("MainTask",
        {
            d.ImuWorker,
            d.AlterImuWorker,
            d.MotorWorker
        });

    //创建推理任务列表，并添加worker，设置推理任务频率
    d.DanceNetInferWorker = d.TaskScheduler->template CreateWorker<BeyondMimicUnitreeInferWorkerType>(cfg_workers["DanceNet1"], cfg_workers["DanceNet1"], JOINT_ID_MAP);
    d.TaskScheduler->CreateTaskList("DanceInferTask", cfg_root["Scheduler"]["InferTask"]["PolicyFrequency"]);
    d.TaskScheduler->AddWorkers("DanceInferTask",
        {
            d.DanceNetInferWorker,
            d.ActionManagementWorker//,
            //d.Logger
        });

    //创建走路的推理任务列表，并添加worker，设置推理任务频率
    d.WalkNetInferWorker = d.TaskScheduler->template CreateWorker<UnitreeRlLabVelocityInferWorkerType>(cfg_workers["WalkNet1"], cfg_workers["MotorControl"]);
    d.WalkCmdWorker = d.TaskScheduler->template CreateWorker<CmdWorkerType>(cfg_workers["WalkCmd"]);
    d.TaskScheduler->CreateTaskList("InferWalkTask", cfg_root["Scheduler"]["InferTask"]["PolicyFrequency"]);
    d.TaskScheduler->AddWorkers("InferWalkTask",
        {
            d.WalkCmdWorker,
            d.WalkNetInferWorker,
            d.ActionManagementWorker,
            d.Logger
        });

    //创建复位任务列表，并添加worker，设置复位任务频率为主任务频率的1/10
    d.MotorResetWorker = d.TaskScheduler->template CreateWorker<MotorResetWorkerType>(cfg_workers["MotorControl"], cfg_workers["ResetPosition"]);
    d.TaskScheduler->CreateTaskList("ResetTask", 10);
    d.TaskScheduler->AddWorker("ResetTask", d.MotorResetWorker);

    //开始调度器
    d.TaskScheduler->Start();
}

void FinishFunc(UserData& d)
{
    std::cout << "Finishing user functions..." << std::endl;
    std::cout << "Goodbye!" << std::endl;
}


void StateWaitingFunc(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& d)
{
    //空闲等待状态，重置目标位置防止突变
    d.MotorWorker->SetCurrentPositionAsTargetPosition();
    //调度器进行一次调度
    d.TaskScheduler->SpinOnce();
}

void StateSystemTestFunc(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& user_data)
{
    //bitbot测试状态为空，用户可自行添加
}


void StatePolicyRunFunc(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& d)
{
    d.TaskScheduler->SpinOnce(); //运行状态(控制主状态)，进行一次调度
};

void StatePolicyDanceFunc(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& d)
{
    d.TaskScheduler->SpinOnce(); //运行状态(跳舞主状态)，进行一次调度
}

void StateInitPoseFunc(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& d)
{
    d.TaskScheduler->SpinOnce(); //复位状态，进行一次调度
}




