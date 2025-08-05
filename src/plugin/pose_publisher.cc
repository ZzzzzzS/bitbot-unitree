#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <rclcpp/rclcpp.hpp>

class PosePublisher : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate {
 public:
  PosePublisher() = default;
  ~PosePublisher() = default;

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &event_mgr) override {
    // Get model name from SDF plugin config
    model_name_ = sdf->Get<std::string>("model_name");
    link_name_ = sdf->Get<std::string>("link_name");
    topic_name_ = sdf->Get<std::string>("topic_name");

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    ros_node_ = rclcpp::Node::make_shared("gz_pose_publisher");
    pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_name_, 10);

    gzmsg << "PosePublisher plugin initialized for model: " << model_name_
          << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) {
      return;
    }

    // Get model entity if enpty
    if (model_entity_ == gz::sim::kNullEntity) {
      ecm.Each<gz::sim::components::Name, gz::sim::components::Model>(
          [&](const gz::sim::Entity &entity,
              const gz::sim::components::Name *name,
              const gz::sim::components::Model *model) -> bool {
            if (name->Data() == model_name_) {
              this->model_entity_ = entity;
              gzmsg << "Found model entity: " << model_name_ << std::endl;
              return false;
            }
            return true;
          });

      if (model_entity_ == gz::sim::kNullEntity) {
        gzerr << "Model entity with name '" << model_name_ << "' not found."
              << std::endl;
        return;
      }
    }

    // Get link entity if empty
    if (link_entity_ == gz::sim::kNullEntity) {
      // auto child_entities =
      //     ecm.ChildrenByComponents(model_entity_,
      //     gz::sim::components::Link());
      ecm.Each<gz::sim::components::Link, gz::sim::components::ParentEntity,
               gz::sim::components::Name>(
          [&](const gz::sim::Entity &child_entity,
              const gz::sim::components::Link *link,
              const gz::sim::components::ParentEntity *parent,
              const gz::sim::components::Name *name) -> bool {
            if (parent->Data() == model_entity_ && name->Data() == link_name_) {
              link_entity_ = child_entity;
              gzmsg << "Found link entity: " << link_name_ << std::endl;
              return false;  // Stop iterating once we find the link
            }
            return true;  // Continue iterating
          });

      if (model_entity_ == gz::sim::kNullEntity) {
        gzerr << "Link entity with name '" << link_name_ << "' not found."
              << std::endl;
        return;
      }
    }

    auto model_pose_component =
        ecm.Component<gz::sim::components::Pose>(model_entity_);
    // Link pose is relative to model pose, so we need both
    auto link_pose_component =
        ecm.Component<gz::sim::components::Pose>(link_entity_);
    if (!link_pose_component || !model_pose_component) {
      return;
    }
    auto link_world_pose =
        model_pose_component->Data() * link_pose_component->Data();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = rclcpp::Clock().now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = link_world_pose.Pos().X();
    pose_msg.pose.position.y = link_world_pose.Pos().Y();
    pose_msg.pose.position.z = link_world_pose.Pos().Z();
    pose_msg.pose.orientation.x = link_world_pose.Rot().X();
    pose_msg.pose.orientation.y = link_world_pose.Rot().Y();
    pose_msg.pose.orientation.z = link_world_pose.Rot().Z();
    pose_msg.pose.orientation.w = link_world_pose.Rot().W();

    this->pose_pub_->publish(pose_msg);
  }

 private:
  std::string model_name_;
  std::string link_name_{"base_link"};
  std::string topic_name_{"/robot_pose_gt"};

  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

#include <gz/plugin/Register.hh>
GZ_ADD_PLUGIN(PosePublisher, gz::sim::System, PosePublisher::ISystemConfigure,
              PosePublisher::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(PosePublisher, "gz::sim::systems::PosePublisherPlugin")
