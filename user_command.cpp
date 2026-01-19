/**
 * @file user_command.cpp
 * @author zishun zhou
 * @brief
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "user_command.h"
#include "user_func.h"


std::optional<bitbot::StateId> EventInitPoseFunc(bitbot::EventValue value, UserData& d)
{
    if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    {
        d.MotorResetWorker->StartReset(); //开始复位
        d.TaskScheduler->EnableTaskList("ResetTask"); //在复位任务列表中启用复位任务
        return static_cast<bitbot::StateId>(States::StateInitPose);
    }
    return std::optional<bitbot::StateId>();
}


std::optional<bitbot::StateId> EventPolicyRunFunc(bitbot::EventValue value, UserData& d)
{
    if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    {
        std::cout << "policy run\n";
        d.MotorResetWorker->StopReset(); //停止复位
        d.TaskScheduler->DisableTaskList("ResetTask"); //在复位任务列表中禁用复位任务
        d.ActionManagementWorker->template SwitchTo<WalkNet1OutPair>();
        d.TaskScheduler->EnableTaskList("InferWalkTask"); //在推理任务列表中启用推理任务
        d.TaskScheduler->EnableTaskList("DanceInferTask"); //在推理任务列表中启用推理任务

        d.TaskScheduler->CreateTimedCallback([](SchedulerType::Ptr scheduler) {
            std::cout << "DanceTaskList is stopped." << std::endl;
            scheduler->DisableTaskList("DanceInferTask");
            }, 3.0 / d.TaskScheduler->getSpinOnceTime());

        return static_cast<bitbot::StateId>(States::StatePolicyRun);
    }
    return std::optional<bitbot::StateId>();
}

std::optional<bitbot::StateId> EventPolicyDanceFunc(bitbot::EventValue value, UserData& d)
{
    if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    {
        std::cout << "policy Dance\n";
        d.MotorResetWorker->StopReset(); //停止复位
        d.TaskScheduler->DisableTaskList("ResetTask"); //在复位任务列表中禁用复位任务
        d.ActionManagementWorker->template SwitchTo<DanceNet1OutPair>();
        d.DanceNetInferWorker->reset();
        d.TaskScheduler->EnableTaskList("InferWalkTask"); //在推理任务列表中启用推理任务
        d.TaskScheduler->EnableTaskList("DanceInferTask"); //在推理任务列表中启用推理任务

        d.TaskScheduler->CreateTimedCallback([](SchedulerType::Ptr scheduler) {
            std::cout << "WalkTask is stopped." << std::endl;
            scheduler->DisableTaskList("InferWalkTask");
            }, 3.0 / d.TaskScheduler->getSpinOnceTime());

        return static_cast<bitbot::StateId>(States::StatePolicyDance);
    }
    return std::optional<bitbot::StateId>();
}


std::optional<bitbot::StateId> EventSystemTestFunc(bitbot::EventValue value,
    UserData& user_data)
{
    //进入bitbot测试状态
    if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    {
        return static_cast<bitbot::StateId>(States::StateSystemTest);
    }
    return std::optional<bitbot::StateId>();
}

std::optional<bitbot::StateId> EventStartDanceFunc(bitbot::EventValue value, UserData& user_data)
{
    if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    {
        //触发跳舞信号
        std::cout << "dance\n";
        user_data.DanceNetInferWorker->start();
    }
    return std::optional<bitbot::StateId>();
}
