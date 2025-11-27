#include "device/gz_robot.h"

namespace bitbot
{
    UnitreeMotherboard::UnitreeMotherboard(pugi::xml_node const& device_node)
        :GzDevice(device_node)
    {
        basic_type_ = (uint32_t)BasicDeviceType::SENSOR;
        type_ = (uint32_t)GzDeviceType::GZ_MOTHERBOARD;
        monitor_header_.headers = { "current","voltage","temperature" };
        monitor_data_.resize(monitor_header_.headers.size());
    }


    UnitreeMotherboard::~UnitreeMotherboard() {}

    void UnitreeMotherboard::Input(const IOType& IO)
    {
        this->Motherboard = std::get<unitree_hg::msg::dds_::MainBoardState_>(IO);
    }

    IOType UnitreeMotherboard::Output()
    {
        return IOType();
    }

    void UnitreeMotherboard::UpdateRuntimeData()
    {
    }
};
