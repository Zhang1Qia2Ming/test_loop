cmake_minimum_required(VERSION 3.14)
project(test_loop VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Box2D: 关闭 testbed 和 unit-test 以加速编译
set(BOX2D_BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_BUILD_TESTBED OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/box2d)

# 核心库
add_library(${PROJECT_NAME}
    src/physics/world.cpp
    src/physics/vehicle.cpp
    src/sensor/raycast_lidar.cpp
    src/simulator.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC box2d)

if(NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 示例
add_subdirectory(examples/minimal_parking)
add_subdirectory(examples/pid_tracking)
add_subdirectory(examples/visualization)

# 测试
enable_testing()
add_subdirectory(tests)
