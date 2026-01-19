/**
 * @file main.cpp
 * @author zishun zhou (zhouzishun@mail.zzshub.cn)
 * @brief
 *
 * @date 2025-03-04
 *
 * @copyright Copyright (c) 2025
 */

#include "user_func.h"
#include "user_command.h"


int main(int argc, char const* argv[])
{
    //NOTE: 注意将配置文件路径修改为自己的路径
#ifdef BUILD_SIMULATION
    std::string cfg_path = PROJECT_ROOT_DIR + std::string("/settings/bitbot_mujoco.xml");
#else
    std::string cfg_path = PROJECT_ROOT_DIR + std::string("/settings/bitbot_unitree.xml");
#endif
    KernelType kernel(cfg_path);

    kernel.RegisterConfigFunc(ConfigFunc);
    kernel.RegisterFinishFunc(FinishFunc);

    // 注册 Event
    kernel.RegisterEvent("system_test", static_cast<bitbot::EventId>(Events::EventSystemTest), &EventSystemTestFunc);
    kernel.RegisterEvent("init_pose", static_cast<bitbot::EventId>(Events::EventInitPose), &EventInitPoseFunc);
    kernel.RegisterEvent("policy_run", static_cast<bitbot::EventId>(Events::EventPolicyRun), &EventPolicyRunFunc);
    kernel.RegisterEvent("policy_dance", static_cast<bitbot::EventId>(Events::EventPolicyDance), &EventPolicyDanceFunc);

    //跳舞状态下的事件
    kernel.RegisterEvent("start_dance", static_cast<bitbot::EventId>(DanceEvnets::EventStartDance), &EventStartDanceFunc);

    // 注册用户命令事件(键盘/手柄按钮)
    kernel.RegisterEvent("velo_x_increase", static_cast<bitbot::EventId>(UserCmdEvents::VeloxIncrease), &EventCmdNIncrease<0>);
    kernel.RegisterEvent("velo_y_increase", static_cast<bitbot::EventId>(UserCmdEvents::VeloyIncrease), &EventCmdNIncrease<1>);
    kernel.RegisterEvent("velo_yaw_increase", static_cast<bitbot::EventId>(UserCmdEvents::VeloYawIncrease), &EventCmdNIncrease<2>);
    // kernel.RegisterEvent("pos_z_increase", static_cast<bitbot::EventId>(UserCmdEvents::PosZIncrease), &EventCmdNIncrease<3>);
    // kernel.RegisterEvent("pos_pitch_increase", static_cast<bitbot::EventId>(UserCmdEvents::PosPitchIncrease), &EventCmdNIncrease<4>);
    // kernel.RegisterEvent("pos_roll_increase", static_cast<bitbot::EventId>(UserCmdEvents::PosRollIncrease), &EventCmdNIncrease<5>);

    kernel.RegisterEvent("velo_x_decrease", static_cast<bitbot::EventId>(UserCmdEvents::VeloxDecrease), &EventCmdNDecrease<0>);
    kernel.RegisterEvent("velo_y_decrease", static_cast<bitbot::EventId>(UserCmdEvents::VeloyDecrease), &EventCmdNDecrease<1>);
    kernel.RegisterEvent("velo_yaw_decrease", static_cast<bitbot::EventId>(UserCmdEvents::VeloYawDecrease), &EventCmdNDecrease<2>);
    // kernel.RegisterEvent("pos_z_decrease", static_cast<bitbot::EventId>(UserCmdEvents::PosZDecrease), &EventCmdNDecrease<3>);
    // kernel.RegisterEvent("pos_pitch_decrease", static_cast<bitbot::EventId>(UserCmdEvents::PosPitchDecrease), &EventCmdNDecrease<4>);
    // kernel.RegisterEvent("pos_roll_decrease", static_cast<bitbot::EventId>(UserCmdEvents::PosRollDecrease), &EventCmdNDecrease<5>);

    // 注册用连续命令事件(手柄摇杆)
    kernel.RegisterEvent("gamepad_x_move", static_cast<bitbot::EventId>(UserCmdEvents::GamepadXMove), &EventCmdNMove<0>);
    kernel.RegisterEvent("gamepad_y_move", static_cast<bitbot::EventId>(UserCmdEvents::GamepadYMove), &EventCmdNMove<1>);
    kernel.RegisterEvent("gamepad_yaw_move", static_cast<bitbot::EventId>(UserCmdEvents::GamepadYawMove), &EventCmdNMove<2>);
    // kernel.RegisterEvent("gamepad_z_move", static_cast<bitbot::EventId>(UserCmdEvents::GamepadZMove), &EventCmdNMove<3>);
    // kernel.RegisterEvent("gamepad_pitch_move", static_cast<bitbot::EventId>(UserCmdEvents::GamepadPitchMove), &EventCmdNMove<4>);
    // kernel.RegisterEvent("gamepad_roll_move", static_cast<bitbot::EventId>(UserCmdEvents::GamepadRollMove), &EventCmdNMove<5>);


    // 注册 State
    kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::StateWaiting),
        &StateWaitingFunc,
        { static_cast<bitbot::EventId>(Events::EventSystemTest), (Events::EventInitPose) });

    kernel.RegisterState("SystemTest", static_cast<bitbot::StateId>(States::StateSystemTest), &StateSystemTestFunc, {});

    kernel.RegisterState("init_pose",
        static_cast<bitbot::StateId>(States::StateInitPose),
        &StateInitPoseFunc,
        { (Events::EventPolicyRun),(Events::EventPolicyDance) });


    std::vector<bitbot::EventId> PolicyRunEvents;
    for (size_t i = UserCmdEvents::VeloxIncrease; i < UserCmdEvents::USER_CMD_EVENT_END; i++)
    {
        PolicyRunEvents.push_back(static_cast<bitbot::EventId>(i));
    }
    PolicyRunEvents.push_back(static_cast<bitbot::EventId>(Events::EventPolicyDance));
    PolicyRunEvents.push_back(static_cast<bitbot::EventId>(Events::EventPolicyRun));
    kernel.RegisterState("policy_run", static_cast<bitbot::StateId>(States::StatePolicyRun), &StatePolicyRunFunc, PolicyRunEvents);

    std::vector<bitbot::EventId> PolicyDanceEvents;
    for (size_t i = DanceEvnets::EventStartDance; i < DanceEvnets::DANCE_EVENT_END; i++)
    {
        PolicyDanceEvents.push_back(static_cast<bitbot::EventId>(i));
    }
    PolicyDanceEvents.push_back(static_cast<bitbot::EventId>(Events::EventPolicyDance));
    PolicyDanceEvents.push_back(static_cast<bitbot::EventId>(Events::EventPolicyRun));
    kernel.RegisterState("policy_dance", static_cast<bitbot::StateId>(States::StatePolicyDance), &StatePolicyDanceFunc, PolicyDanceEvents);

    kernel.SetFirstState(static_cast<bitbot::StateId>(States::StateWaiting));
    kernel.Run(); // Run the kernel
    return 0;
}
