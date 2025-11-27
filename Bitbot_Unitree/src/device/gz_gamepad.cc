#include "device/gz_gamepad.h"
#include "bitbot_kernel/kernel/kernel_interface.hpp"

namespace bitbot
{
    UnitreeGamepad::UnitreeGamepad(pugi::xml_node const& device_node)
        :GzDevice(device_node)
    {
        this->basic_type_ = (uint32_t)BasicDeviceType::SENSOR;
        this->type_ = (uint32_t)GzDeviceType::GZ_GAMEPAD;

        this->monitor_header_.headers = {
            "R1",
            "L1",
            "start",
            "select",
            "R2",
            "L2",
            "F1",
            "F2",
            "A",
            "B",
            "X",
            "Y",
            "up",
            "right",
            "down",
            "left",
            "lx",
            "rx",
            "ry",
            "L2",
            "ly"
        };
        monitor_data_.resize(monitor_header_.headers.size());

        double dead_zone, smooth;
        ConfigParser::ParseAttribute2d(dead_zone, device_node.attribute("dead_zone"));
        ConfigParser::ParseAttribute2d(smooth, device_node.attribute("smooth"));
        this->gamepad_ = new Gamepad(smooth, dead_zone);
    }

    UnitreeGamepad::~UnitreeGamepad()
    {
        delete this->gamepad_;
    }

    void UnitreeGamepad::ProcessButtonEvent(const Button& button, const std::string& key)
    {
        if (button.on_toggle && this->EventMap.count(key) != 0)
        {
            this->kernel_interface->EmitEvent(this->EventMap[key], button.pressed);
        }
    }

    void UnitreeGamepad::ProcessJoystickEvent(float joystick, const std::string& key)
    {
        if (this->EventMap.count(key) != 0)
        {
            this->kernel_interface->EmitEvent(this->EventMap[key], joystick * 32768.0);
        }
    }

    void UnitreeGamepad::Input(const IOType& IO)
    {
        auto low_state = std::get<REMOTE_DATA_RX>(IO);
        this->gamepad_->update(low_state.RF_RX);

        this->ProcessButtonEvent(this->gamepad_->R1, "GAMEPAD_BUTTON_RB");
        this->ProcessButtonEvent(this->gamepad_->L1, "GAMEPAD_BUTTON_LB");
        this->ProcessButtonEvent(this->gamepad_->start, "GAMEPAD_BUTTON_MENU");
        this->ProcessButtonEvent(this->gamepad_->select, "GAMEPAD_BUTTON_SWITCH");
        this->ProcessButtonEvent(this->gamepad_->R2, "GAMEPAD_JOYSTICK_RT");
        this->ProcessButtonEvent(this->gamepad_->L2, "GAMEPAD_JOYSTICK_LT");
        this->ProcessButtonEvent(this->gamepad_->F1, "GAMEPAD_BUTTON_L_STICK");
        this->ProcessButtonEvent(this->gamepad_->F2, "GAMEPAD_BUTTON_R_STICK");
        this->ProcessButtonEvent(this->gamepad_->A, "GAMEPAD_BUTTON_A");
        this->ProcessButtonEvent(this->gamepad_->B, "GAMEPAD_BUTTON_B");
        this->ProcessButtonEvent(this->gamepad_->X, "GAMEPAD_BUTTON_X");
        this->ProcessButtonEvent(this->gamepad_->Y, "GAMEPAD_BUTTON_Y");
        this->ProcessButtonEvent(this->gamepad_->up, "GAMEPAD_HAT_UP");
        this->ProcessButtonEvent(this->gamepad_->right, "GAMEPAD_HAT_RIGHT");
        this->ProcessButtonEvent(this->gamepad_->down, "GAMEPAD_HAT_DOWN");
        this->ProcessButtonEvent(this->gamepad_->left, "GAMEPAD_HAT_LEFT");

        this->ProcessJoystickEvent(this->gamepad_->lx, "GAMEPAD_JOYSTICK_LX");
        this->ProcessJoystickEvent(this->gamepad_->rx, "GAMEPAD_JOYSTICK_RX");
        this->ProcessJoystickEvent(this->gamepad_->ry, "GAMEPAD_JOYSTICK_RY");
        this->ProcessJoystickEvent(this->gamepad_->ly, "GAMEPAD_JOYSTICK_LY");

    }

    IOType UnitreeGamepad::Output()
    {
        return IOType();
    }

    void UnitreeGamepad::UpdateRuntimeData()
    {
        monitor_data_[0] = static_cast<double>(this->gamepad_->R1.pressed);
        monitor_data_[1] = static_cast<double>(this->gamepad_->L1.pressed);
        monitor_data_[2] = static_cast<double>(this->gamepad_->start.pressed);
        monitor_data_[3] = static_cast<double>(this->gamepad_->select.pressed);
        monitor_data_[4] = static_cast<double>(this->gamepad_->R2.pressed);
        monitor_data_[5] = static_cast<double>(this->gamepad_->L2.pressed);
        monitor_data_[6] = static_cast<double>(this->gamepad_->F1.pressed);
        monitor_data_[7] = static_cast<double>(this->gamepad_->F2.pressed);
        monitor_data_[8] = static_cast<double>(this->gamepad_->A.pressed);
        monitor_data_[9] = static_cast<double>(this->gamepad_->B.pressed);
        monitor_data_[10] = static_cast<double>(this->gamepad_->X.pressed);
        monitor_data_[11] = static_cast<double>(this->gamepad_->Y.pressed);
        monitor_data_[12] = static_cast<double>(this->gamepad_->up.pressed);
        monitor_data_[13] = static_cast<double>(this->gamepad_->right.pressed);
        monitor_data_[14] = static_cast<double>(this->gamepad_->down.pressed);
        monitor_data_[15] = static_cast<double>(this->gamepad_->left.pressed);
        monitor_data_[16] = static_cast<double>(this->gamepad_->lx);
        monitor_data_[17] = static_cast<double>(this->gamepad_->rx);
        monitor_data_[18] = static_cast<double>(this->gamepad_->ly);
        monitor_data_[19] = static_cast<double>(this->gamepad_->l2);
        monitor_data_[20] = static_cast<double>(this->gamepad_->ly);
    }

};


