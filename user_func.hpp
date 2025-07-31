#ifndef USER_FUNC_HPP
#define USER_FUNC_HPP

#include "bitbot_gz/kernel/gz_kernel.hpp"
#include "rclcpp/rclcpp.hpp"

struct UserData {};

enum Events {
  Wait = 1001,
};

enum States : bitbot::StateId {
  Waiting = 1001,
};

using Kernel = bitbot::GzKernel<UserData>;

inline std::optional<bitbot::StateId> EventWait(bitbot::EventValue value,
                                                UserData &user_data) {
  return static_cast<bitbot::StateId>(States::Waiting);
}

inline void StateWaiting(const bitbot::KernelInterface &kernel,
                         Kernel::ExtraData &extra_data, UserData &user_data) {}

inline void ConfigFunc(const bitbot::GzBus &bus, UserData &) {}

inline void FinishFunc(UserData &) {}

#endif  // !USER_FUNC_HPP
