cmake_minimum_required(VERSION 3.14)

project(bitbot)

set(BITBOT_DEPENDENCY_USE_PROXY OFF)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/dependencies)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


add_subdirectory(CtrlZ)
include(bitbot.cmake)

option(BUILD_SIMULATION "Build simulation" ON)

if(BUILD_SIMULATION)
  add_definitions(-DBUILD_SIMULATION)
  BitbotAddDependency(
    NAME bitbot_mujoco
    FILE_NAME "bitbot_mujoco-main.zip"
    FILE_PATH ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
    USE_LOCAL_FILE TRUE
  )
else()
  add_subdirectory(Bitbot_Unitree)
  BitbotAddDependency(
    NAME bitbot_kernel
    FILE_NAME "bitbot_kernel-main.zip"
    FILE_PATH ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
    USE_LOCAL_FILE TRUE
  )
endif()


set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DPROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}")
message("PROJECT_ROOT_DIR: ${PROJECT_ROOT_DIR}")

add_executable(main_app
  main.cpp
  types.hpp
  user_func.h
  user_func.cpp
  user_command.h
  user_command.cpp
)

if(BUILD_SIMULATION)
  target_link_libraries(main_app BitbotMujoco)
else()
  target_link_libraries(main_app bitbotUnitree)
endif()

target_link_libraries(main_app
  BitbotKernel
  CtrlZ
)

target_compile_options(main_app PUBLIC)
target_compile_features(main_app PUBLIC cxx_std_20)
