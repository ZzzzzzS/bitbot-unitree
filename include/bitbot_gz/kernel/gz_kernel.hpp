#ifndef GZ_KERNEL_HPP
#define GZ_KERNEL_HPP

#include <thread>

#include "bitbot_gz/bus/gz_bus.h"
#include "bitbot_gz/kernel/ros_interface.hpp"
#include "bitbot_kernel/kernel/kernel.hpp"

namespace bitbot {
template <typename UserData, CTString... cts>
class GzKernel
    : public KernelTpl<GzKernel<UserData, cts...>, GzBus, UserData, cts...> {
 public:
  GzKernel(std::string config_file)
      : KernelTpl<GzKernel<UserData, cts...>, GzBus, UserData, cts...>(
            config_file) {
    // TODO
    pugi::xml_node const& bitbot_node = this->parser_->GetBitbotNode();

    ros_interface_ = std::make_shared<RosInterface>();
    RosInterface::RunRosSpin(ros_interface_);

    this->busmanager_.SetInterface(ros_interface_);
  }

  ~GzKernel() = default;

 private:
  void doStart() {
    RCLCPP_INFO(rclcpp::get_logger("bitbot kernel"), "Kernel started.");
  }

  void doRun() {
    // this->busmanager_.UpdateDevices();
    std::chrono::high_resolution_clock::time_point this_time =
        std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point last_time = this_time;
    std::chrono::high_resolution_clock::time_point end_time = this_time;

    while (rclcpp::ok()) {
      while (ros_interface_->IsClockReady()) {
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }

      this->kernel_runtime_data_.kernel_loop_count++;
      this->kernel_runtime_data_.periods_count++;
      this_time = std::chrono::high_resolution_clock::now();
      this->kernel_runtime_data_.period =
          std::chrono::duration_cast<std::chrono::nanoseconds>(this_time -
                                                               last_time)
              .count() /
          1e6;
      last_time = this_time;

      this->HandleEvents();
      this->KernelLoopTask();
      end_time = std::chrono::high_resolution_clock::now();
      this->kernel_runtime_data_.process_time =
          std::chrono::duration_cast<std::chrono::nanoseconds>(end_time -
                                                               this_time)
              .count() /
          1e6;

      this->KernelPrivateLoopEndTask();
    }
  }

  void PowerOn() { this->kernel_config_data_.power_on_finish_flag = true; }

 private:
  RosInterface::Ptr ros_interface_;
};
}  // namespace bitbot

#endif  // !GZ_KERNEL_HPP
