#ifndef GZ_KERNEL_HPP
#define GZ_KERNEL_HPP

#include <thread>

#include "bus/gz_bus.h"
#include "bitbot_kernel/kernel/kernel.hpp"

namespace bitbot {
  /// @brief Bitbot unitree 内核事件类型，继承自EventId
  enum class UnitreeKernelEvent : EventId
  {
    POWER_ON = 200
  };

  /// @brief Bitbot unitree 内核状态类型，继承自StateId
  enum class UnitreeKernelState : StateId
  {
    POWER_ON_FINISH = 100
  };


  template <typename UserData>
  class GzKernel
    : public KernelTpl<GzKernel<UserData>, GzBus, UserData> {
  public:
    GzKernel(std::string config_file)
      : KernelTpl<GzKernel<UserData>, GzBus, UserData>(
        config_file) {
      pugi::xml_node UnitreeKernel_node = this->parser_->GetBitbotNode();
      pugi::xml_node Unitree_node = UnitreeKernel_node.child("Unitree");
      int bus_freq;
      ConfigParser::ParseAttribute2i(bus_freq, Unitree_node.attribute("BusFrequency"));
      this->run_period = 1e6 / bus_freq;
      this->busmanager_.Init(Unitree_node);

      this->KernelRegisterEvent("power_on", static_cast<EventId>(UnitreeKernelEvent::POWER_ON), [this](EventValue, UserData&)
        {
          this->logger_->info("joints power on");
          this->busmanager_.PowerOn();
          this->logger_->info("joints power on finished");
          return static_cast<StateId>(UnitreeKernelState::POWER_ON_FINISH); }, false);

      this->KernelRegisterState("power on finish", static_cast<StateId>(UnitreeKernelState::POWER_ON_FINISH),
        [this](const bitbot::KernelInterface& kernel, ExtraData& extra_data, UserData& user_data) {}, { static_cast<EventId>(KernelEvent::START) });

      this->InjectEventsToState(static_cast<StateId>(KernelState::IDLE), { static_cast<EventId>(UnitreeKernelEvent::POWER_ON) });

      this->PrintWelcomeMessage(); // MUST PRIENT WELCOME MESSAGE!!!!!!
    }

    ~GzKernel() = default;

  public:
    void doStart() {
      this->logger_->info("Bitbot Unitree Kernel started.");
    }

    void doRun() {
      std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
      std::chrono::high_resolution_clock::time_point last_time = start_time;
      std::chrono::high_resolution_clock::time_point end_time = start_time;
      constexpr float ns_to_ms = 1 / 1e6;
      constexpr float ms_to_ms = 1 / 1e3;
      constexpr float s_to_ms = 1e3;

      while (!this->busmanager_.isSystemReady()) {
        this->logger_->info("Waiting for unitree system to be ready...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      this->logger_->info("unitree system is ready.");

      while (!this->kernel_config_data_.stop_flag)
      {
        start_time = std::chrono::high_resolution_clock::now();
        this->kernel_runtime_data_.periods_count++;
        this->kernel_runtime_data_.period = std::chrono::duration_cast<std::chrono::microseconds>(start_time - last_time).count() * ms_to_ms;
        last_time = start_time;

        this->HandleEvents();
        this->KernelLoopTask();
        this->KernelPrivateLoopEndTask();

        end_time = std::chrono::high_resolution_clock::now();

        auto time_cost = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        this->kernel_runtime_data_.process_time = std::chrono::duration_cast<std::chrono::microseconds>(time_cost).count() * ms_to_ms;

        auto sleep_time = std::chrono::microseconds(this->run_period) - time_cost;
        if (sleep_time > std::chrono::microseconds(0)) [[likely]]
        {
          std::this_thread::sleep_for(sleep_time);
        }
        else
        {
          if (this->kernel_runtime_data_.periods_count > 1000) [[likely]]
            this->logger_->warn("program time out!");
        }
      }

      this->logger_->info("Bitbot Unitree Kernel stopped.");
    }

  private:
    void PrintWelcomeMessage()
    {
      std::string line0 = "\033[32m========================================================================================\033[0m";
      std::string line1 = "\033[32m|| BBBB   III  TTTTT  BBBB    OOO   TTTTT     U   U N   N III TTTTT RRRR  EEEEE EEEEE ||\033[0m";
      std::string line2 = "\033[32m|| B   B   I     T    B   B  O   O    T       U   U NN  N  I    T   R   R E     E     ||\033[0m";
      std::string line3 = "\033[32m|| BBBB    I     T    BBBB   O   O    T       U   U N N N  I    T   RRRR  EEE   EEE   ||\033[0m";
      std::string line4 = "\033[32m|| B   B   I     T    B   B  O   O    T       U   U N  NN  I    T   R  R  E     E     ||\033[0m";
      std::string line5 = "\033[32m|| BBBB   III    T    BBBB    OOO     T        UUU  N   N III   T   R   R EEEEE EEEEE ||\033[0m";
      std::string line6 = "\033[32m========================================================================================\033[0m";

      std::cout << std::endl << std::endl;
      std::cout << "\033[31mWelcome to use Bitbot Unitree. Make Bitbot Everywhere! \033[0m" << std::endl;
      std::cout << line0 << std::endl;
      std::cout << line1 << std::endl;
      std::cout << line2 << std::endl;
      std::cout << line3 << std::endl;
      std::cout << line4 << std::endl;
      std::cout << line5 << std::endl;
      std::cout << line6 << std::endl << std::endl;
    }
  private:
    int run_period; // run period in micro second
  };
}  // namespace bitbot

#endif  // !GZ_KERNEL_HPP
