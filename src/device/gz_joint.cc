#include "bitbot_gz/device/gz_joint.h"

namespace bitbot {

GzJoint::GzJoint(const pugi::xml_node& device_node) : GzDevice(device_node) {
  //
}

GzJoint::~GzJoint() {}

void GzJoint::UpdateModel() {}

void Input() {}

void Output() {}

void UpdateRuntimeData() {}

}  // namespace bitbot
