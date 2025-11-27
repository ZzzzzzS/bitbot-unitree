#ifndef GZ_ROBOT_H
#define GZ_ROBOT_H

#include "device/gz_device.hpp"

namespace bitbot {

    class UnitreeMotherboard final : public GzDevice {
    public:
        UnitreeMotherboard(pugi::xml_node const& device_node);
        ~UnitreeMotherboard();

    private:
        virtual void Input(const IOType& IO) final;
        virtual IOType Output() final;
        virtual void UpdateRuntimeData() final;

        unitree_hg::msg::dds_::MainBoardState_ Motherboard;
    };

}  // namespace bitbot

#endif  // !GZ_BATTERY_H
