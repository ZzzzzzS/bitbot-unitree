#include "bitbot_gz/device/gz_imu.h"

namespace bitbot {

GzImu::GzImu(pugi::xml_node const& device_node) : GzDevice(device_node) {
  //
}

GzImu::~GzImu() {
  //
}

void UpdateModel() {}

void Input() {}

void Output() {}

void UpdateRuntimeData() {}

}  // namespace bitbot
