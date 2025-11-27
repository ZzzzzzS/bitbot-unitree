#include "device/gz_battery.h"

namespace bitbot
{
    UnitreeBattery::UnitreeBattery(pugi::xml_node const& device_node)
        :GzDevice(device_node)
    {
        basic_type_ = (uint32_t)BasicDeviceType::SENSOR;
        type_ = (uint32_t)GzDeviceType::GZ_BATTERY;
        monitor_header_.headers = { "current","voltage","temperature" };
        monitor_data_.resize(monitor_header_.headers.size());
    }


    UnitreeBattery::~UnitreeBattery() {}

    void UnitreeBattery::Input(const IOType& IO)
    {
        this->battery = std::get<unitree_hg::msg::dds_::BmsState_>(IO);
    }

    IOType UnitreeBattery::Output()
    {
        return IOType();
    }

    void UnitreeBattery::UpdateRuntimeData()
    {
    }
};
