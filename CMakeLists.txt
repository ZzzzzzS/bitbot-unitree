cmake_minimum_required(VERSION 3.8)
project(bitbot_gz)

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

include(FetchContent)
FetchContent_Declare(
  ovinf
  GIT_REPOSITORY git@github.com:Dknt0/ovinf.git 
  GIT_TAG main
)
FetchContent_MakeAvailable(ovinf)

find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
include_directories("/usr/include/eigen3")

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/device BITBOT_GZ_DEVICE_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/bus BITBOT_GZ_BUS_SRC)
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
ament_target_dependencies(bitbot_gz
  rclcpp
  trajectory_msgs
  std_msgs
  sensor_msgs
  rosgraph_msgs
  ament_index_cpp
)
target_link_libraries(bitbot_gz BitbotKernel)

add_executable(main_app main.cc)
target_include_directories(main_app PUBLIC
  include
  ${OVINF_INCLUDED}
  ${OpenVINO_INCLUDE_DIRS}
)
target_link_libraries(main_app
  bitbot_gz
  ovinf
)

add_library(RobotResetPlugin SHARED src/plugin/robot_reset_plugin.cc)
target_link_libraries(RobotResetPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

install(TARGETS
  bitbot_gz main_app
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  RobotResetPlugin
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY launch config urdf rviz world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
