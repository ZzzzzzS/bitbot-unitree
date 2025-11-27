/*****************************************************************//**
 * \file   main.cpp
 * \brief
 *
 * \author ZhouZishun
 * \date   March 2025
 *********************************************************************/
#ifdef BUILD_SIMULATION
# include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#else
#include <sys/mman.h>
#endif //BUILD_SIMULATION
#include "user_func.h"


int main(int argc, char const* argv[])
{
    //NOTE: 注意将配置文件路径修改为自己的路径

#ifdef BUILD_SIMULATION
    std::string cfg_path = PROJECT_ROOT_DIR + std::string("/settings/bitbot_mujoco.xml");
    KernelType kernel(cfg_path);
#else

    std::string cfg_path = PROJECT_ROOT_DIR + std::string("/settings/bitbot_unitree.xml");
    KernelType kernel(cfg_path);
#endif //BUILD_SIMULATION


    kernel.RegisterConfigFunc(ConfigFunc);
    kernel.RegisterFinishFunc(FinishFunc);

    // 注册 Event
    kernel.RegisterEvent("system_test",
        static_cast<bitbot::EventId>(Events::SystemTest),
        &EventSystemTest);

    kernel.RegisterEvent("init_pose",
        static_cast<bitbot::EventId>(Events::InitPose),
        &EventInitPose);
    kernel.RegisterEvent("policy_run",
        static_cast<bitbot::EventId>(Events::PolicyRun),
        &EventPolicyRun);

    // 注册速度控制器
    kernel.RegisterEvent("velo_x_increase", static_cast<bitbot::EventId>(Events::VeloxIncrease), &EventVeloXIncrease);
    kernel.RegisterEvent("velo_x_decrease", static_cast<bitbot::EventId>(Events::VeloxDecrease), &EventVeloXDecrease);
    kernel.RegisterEvent("velo_y_increase", static_cast<bitbot::EventId>(Events::VeloyIncrease), &EventVeloYIncrease);
    kernel.RegisterEvent("velo_y_decrease", static_cast<bitbot::EventId>(Events::VeloyDecrease), &EventVeloYDecrease);


    //注册手柄事件
    kernel.RegisterEvent("gamepad_init_pos", static_cast<bitbot::EventId>(Events::GamepadInitPose), &EventInitPose);
    kernel.RegisterEvent("gamepad_run", static_cast<bitbot::EventId>(Events::GamepadPolicyRun), &EventPolicyRun);
    kernel.RegisterEvent("gamepad_velo_x_increase", static_cast<bitbot::EventId>(Events::GamepadVeloxIncreaseDisc), &EventVeloXIncrease);
    kernel.RegisterEvent("gamepad_velo_x_decrease", static_cast<bitbot::EventId>(Events::GamepadVeloxDecreaseDisc), &EventVeloXDecrease);
    kernel.RegisterEvent("gamepad_velo_y_increase", static_cast<bitbot::EventId>(Events::GamepadVeloyIncreaseDisc), &EventVeloYIncrease);
    kernel.RegisterEvent("gamepad_velo_y_decrease", static_cast<bitbot::EventId>(Events::GamepadVeloyDecreaseDisc), &EventVeloYDecrease);

    // 注册 State
    kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::Waiting),
        &StateWaiting,
        { static_cast<bitbot::EventId>(Events::SystemTest), (Events::InitPose),(Events::GamepadInitPose) });

    kernel.RegisterState("SystemTest", static_cast<bitbot::StateId>(States::PF2SystemTest), &StateSystemTest, {});

    kernel.RegisterState("init_pose",
        static_cast<bitbot::StateId>(States::PF2InitPose),
        &StateJointInitPose,
        { (Events::PolicyRun),(Events::GamepadPolicyRun) });


    kernel.RegisterState("policy_run",
        static_cast<bitbot::StateId>(States::PF2PolicyRun),
        &StatePolicyRun, { static_cast<bitbot::EventId>(Events::VeloxDecrease), static_cast<bitbot::EventId>(Events::VeloxIncrease),
        static_cast<bitbot::EventId>(Events::VeloyDecrease), static_cast<bitbot::EventId>(Events::VeloyIncrease),
        static_cast<bitbot::EventId>(Events::GamepadVeloxIncreaseDisc),static_cast<bitbot::EventId>(Events::GamepadVeloxDecreaseDisc),
        static_cast<bitbot::EventId>(Events::GamepadVeloyIncreaseDisc),static_cast<bitbot::EventId>(Events::GamepadVeloyDecreaseDisc) });

    kernel.SetFirstState(static_cast<bitbot::StateId>(States::Waiting));
    kernel.Run(); // Run the kernel
    return 0;
}
