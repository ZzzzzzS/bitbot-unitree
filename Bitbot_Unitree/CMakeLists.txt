cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(bitbotUnitree)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

aux_source_directory(src/bus SRC_BUS)
aux_source_directory(src/device SRC_DEVICE)

include(FetchContent)
FetchContent_Declare(
    unitree_sdk2
    URL file://${CMAKE_CURRENT_SOURCE_DIR}/pkg/unitree_sdk2.zip
    OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(unitree_sdk2)

find_package(unitree_sdk2 REQUIRED)

message("BUILDING BITBOT UNITREE")
message("MAKE BITBOT EVERY WHERE!")

message("+-----------------------------------------+
|    BBBB   III TTTTT BBBB   OOO  TTTTT   |
|    B   B   I    T   B   B O   O   T     |
|    BBBB    I    T   BBBB  O   O   T     |
|    B   B   I    T   B   B O   O   T     |
|    BBBB   III   T   BBBB   OOO    T     |
|                                         |
| U   U N   N III TTTTT RRRR  EEEEE EEEEE |
| U   U NN  N  I    T   R   R E     E     |
| U   U N N N  I    T   RRRR  EEE   EEE   |
| U   U N  NN  I    T   R  R  E     E     |
|  UUU  N   N III   T   R   R EEEEE EEEEE |
+-----------------------------------------+
")



add_library(bitbotUnitree STATIC
            ./include/kernel/gz_kernel.hpp
            ${SRC_BUS}
            ${SRC_KERNEL}
            ${SRC_DEVICE}
)

target_include_directories(bitbotUnitree PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(bitbotUnitree 
    BitbotKernel
    unitree_sdk2
)