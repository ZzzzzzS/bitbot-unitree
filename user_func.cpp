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

template <typename Container, typename Func>
void Apply(Container& container, Func func) {
    for (size_t i = 0; i < container.size(); ++i) {
        func(&container[i], i);
    }
}


void ConfigFunc(const KernelBus& bus, UserData& d)
{
    //读取json配置文件,并初始化各个worker
    nlohmann::json cfg_root;
    {
        //NOTE: 注意将配置文件路径修改为自己的路径
        std::string path = PROJECT_ROOT_DIR + std::string("/settings/CtrlConfig.json");
        std::ifstream cfg_file(path);
        cfg_root = nlohmann::json::parse(cfg_file, nullptr, true, true);
    }

    //对齐硬件颗粒度
    d.ImuPtr = bus.GetDevice<DeviceImu>(IMU_ID_MAP).value();
    Apply(d.JointsPtr, [&bus](DeviceJoint** joint, size_t i) {
        *joint = bus.GetDevice<DeviceJoint>(JOINT_ID_MAP[i]).value();
        });
}

void FinishFunc(UserData& d)
{
}


std::optional<bitbot::StateId> EventInitPose(bitbot::EventValue value, UserData& d)
{
    return std::nullopt;
}


std::optional<bitbot::StateId> EventPolicyRun(bitbot::EventValue value, UserData& d)
{
    return std::nullopt;
}

std::optional<bitbot::StateId> EventSystemTest(bitbot::EventValue value,
    UserData& user_data)
{
    return std::nullopt;
}

// velocity control callback
#define X_VEL_STEP 0.2
#define Y_VEL_STEP 0.05
std::optional<bitbot::StateId> EventVeloXIncrease(bitbot::EventValue keyState, UserData& d)
{
    return std::nullopt;
}

std::optional<bitbot::StateId> EventVeloXDecrease(bitbot::EventValue keyState, UserData& d)
{
    return std::nullopt;
}

std::optional<bitbot::StateId> EventVeloYIncrease(bitbot::EventValue keyState, UserData& d)
{
    return std::nullopt;
}

std::optional<bitbot::StateId> EventVeloYDecrease(bitbot::EventValue keyState, UserData& d)
{
    return std::nullopt;
}


void StateWaiting(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& d)
{

}

void StateSystemTest(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& user_data)
{
}


void StatePolicyRun(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& d)
{

};


void StateJointInitPose(const bitbot::KernelInterface& kernel,
    bitbot::ExtraData& extra_data, UserData& d)
{
}




