#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

using namespace gz;
using namespace sim;
using namespace systems;

class RobotResetPlugin : public System,
                         public ISystemConfigure,
                         public ISystemPreUpdate,
                         public ISystemUpdate,
                         public ISystemPostUpdate {
  virtual void Configure(const Entity& entity,
                         const std::shared_ptr<const sdf::Element>& sdf,
                         EntityComponentManager& ecm,
                         EventManager& event_mgr) override {}

  virtual void PreUpdate(const UpdateInfo& info,
                         EntityComponentManager& ecm) override {}
  virtual void Update(const UpdateInfo& info,
                      EntityComponentManager& ecm) override {}

  virtual void PostUpdate(const UpdateInfo& info,
                          const EntityComponentManager& ecm) override {}

 private:
};

GZ_ADD_PLUGIN(RobotResetPlugin, gz::sim::System,
              RobotResetPlugin::ISystemConfigure,
              RobotResetPlugin::ISystemPreUpdate,
              RobotResetPlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(RobotResetPlugin, "gz::sim::systems::RobotResetPlugin")
