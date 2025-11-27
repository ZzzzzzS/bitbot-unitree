#ifndef GZ_BATTERY_H
#define GZ_BATTERY_H

#include "device/gz_device.hpp"

namespace bitbot {

    class UnitreeBattery final : public GzDevice {
    public:
        UnitreeBattery(pugi::xml_node const& device_node);
        ~UnitreeBattery();

    private:
        virtual void Input(const IOType& IO) final;
        virtual IOType Output() final;
        virtual void UpdateRuntimeData() final;

        unitree_hg::msg::dds_::BmsState_ battery;
    };

}  // namespace bitbot

#endif  // !GZ_BATTERY_H
