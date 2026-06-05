cmake_minimum_required(VERSION 3.14)
project(test_loop VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 核心库 (目前主要是头文件)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 开启基础编译警告 (不强制 -Werror，避免头文件库对使用者过于严苛)
if(NOT MSVC)
    target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wpedantic)
endif()

# 启用测试
enable_testing()
add_subdirectory(tests)
