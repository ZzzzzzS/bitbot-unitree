#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <nav_msgs/msg/odometry.hpp>
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

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    ros_node_ = rclcpp::Node::make_shared("gz_pose_publisher");
    tf_broadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
    pose_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        "robot_pose_gt", 10);

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
            if (name->Data() == link_name_) {
              link_entity_ = child_entity;
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

    // 200 hz
    if (update_count_++ % 5 != 0) {
      return;
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

    // Use gazebo time instead of ROS time
    geometry_msgs::msg::TransformStamped transform_msg;
    auto sec = info.simTime.count();
    auto nanosec = sec % static_cast<int>(1e9);
    transform_msg.header.stamp.sec = sec / static_cast<int>(1e9);
    transform_msg.header.stamp.nanosec = nanosec;
    transform_msg.header.frame_id = "world";
    transform_msg.child_frame_id = link_name_;

    transform_msg.transform.translation.x = link_world_pose.Pos().X();
    transform_msg.transform.translation.y = link_world_pose.Pos().Y();
    transform_msg.transform.translation.z = link_world_pose.Pos().Z();
    transform_msg.transform.rotation.x = link_world_pose.Rot().X();
    transform_msg.transform.rotation.y = link_world_pose.Rot().Y();
    transform_msg.transform.rotation.z = link_world_pose.Rot().Z();
    transform_msg.transform.rotation.w = link_world_pose.Rot().W();

    this->tf_broadcaster_->sendTransform(transform_msg);

    nav_msgs::msg::Odometry nav_msg;
    nav_msg.header.stamp = transform_msg.header.stamp;
    nav_msg.header.frame_id = "world";
    nav_msg.child_frame_id = link_name_;
    nav_msg.pose.pose.position.x = link_world_pose.Pos().X();
    nav_msg.pose.pose.position.y = link_world_pose.Pos().Y();
    nav_msg.pose.pose.position.z = link_world_pose.Pos().Z();
    nav_msg.pose.pose.orientation.x = link_world_pose.Rot().X();
    nav_msg.pose.pose.orientation.y = link_world_pose.Rot().Y();
    nav_msg.pose.pose.orientation.z = link_world_pose.Rot().Z();
    nav_msg.pose.pose.orientation.w = link_world_pose.Rot().W();
    nav_msg.pose.covariance.fill(0.001);
    this->pose_pub_->publish(nav_msg);
  }

 private:
  std::string model_name_;
  std::string link_name_{"base_link"};

  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};

  rclcpp::Node::SharedPtr ros_node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  size_t update_count_{0};
};

#include <gz/plugin/Register.hh>
GZ_ADD_PLUGIN(PosePublisher, gz::sim::System, PosePublisher::ISystemConfigure,
              PosePublisher::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(PosePublisher, "gz::sim::systems::PosePublisherPlugin")
