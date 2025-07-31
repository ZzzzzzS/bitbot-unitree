cmake_minimum_required(VERSION 3.8)
project(bitbot_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

set(BITBOT_DEPENDENCY_USE_PROXY TRUE)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies)
include(dependencies/bitbot.cmake)
BitbotAddDependency(
  NAME           bitbot_kernel
  FILE_NAME      "bitbot_kernel-main.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_LOCAL_FILE TRUE
)

include_directories("/usr/include/eigen3")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/device BITBOT_GZ_DEVICE_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/bus BITBOT_GZ_BUS_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/kernel BITBOT_GZ_KERNEL_SRC)
set(BITBOT_GZ_SRC
  ${BITBOT_GZ_DEVICE_SRC}
  ${BITBOT_GZ_BUS_SRC}
  ${BITBOT_GZ_KERNEL_SRC}
)

add_library(bitbot_gz ${BITBOT_GZ_SRC})
target_include_directories(bitbot_gz PUBLIC
  $<BUILD_INTERFACE:${THIRD_PARTY_INCLUDE}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bitbot_gz rclcpp trajectory_msgs std_msgs sensor_msgs)
target_link_libraries(bitbot_gz BitbotKernel)

# Torque control demo
add_executable(torque_control src/torque_control.cc)
ament_target_dependencies(torque_control rclcpp trajectory_msgs std_msgs sensor_msgs)

add_executable(main_app main.cc)
target_include_directories(main_app PUBLIC
  include
)
target_link_libraries(main_app
  bitbot_gz
)

install(TARGETS
  torque_control bitbot_gz main_app
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
