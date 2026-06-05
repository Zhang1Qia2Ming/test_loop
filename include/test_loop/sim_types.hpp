#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace test_loop
{

// ========== 控制与状态 ==========

struct ControlCommand
{
    double throttle{0.0};      // 油门开度 [0, 1]
    double brake{0.0};         // 制动强度 [0, 1]
    double steer_angle{0.0};   // 前轮转角 (rad)，左正右负
};

struct PhysicalState
{
    uint64_t timestamp_ms{0};  // 时间戳
    double x{0.0};             // 全局 X (m)
    double y{0.0};             // 全局 Y (m)
    double yaw{0.0};           // 航向角 (rad)
    double v{0.0};             // 纵向速度 (m/s)
    double w{0.0};             // 横摆角速度 (rad/s)
};

constexpr size_t ALGO_STATE_MAX_SIZE = 256;

struct AlgorithmState
{
    std::array<uint8_t, ALGO_STATE_MAX_SIZE> buffer{};
    size_t valid_size{0};      // 实际有效字节数，必须 <= ALGO_STATE_MAX_SIZE
};

struct SnapshotState
{
    PhysicalState physical_state{};
    AlgorithmState algo_state{};
    ControlCommand last_control_command{};
};

// ========== 几何与场景 ==========

struct Pose2D
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

struct VehicleConfig
{
    double wheelbase{2.8};        // 轴距 (m)
    double track_width{1.6};      // 轮距 (m)
    double length{4.5};           // 车长 (m)
    double width{1.8};            // 车宽 (m)
    double front_overhang{0.9};   // 前悬 (m)
    double rear_overhang{0.8};    // 后悬 (m)
    double max_steer_angle{0.52}; // 最大前轮转角 (rad)，约 30°
    double max_speed{10.0};       // 最大车速 (m/s)
    double max_accel{3.0};        // 最大加速度 (m/s^2)
    double max_decel{5.0};        // 最大减速度 (m/s^2)
};

struct Rectangle
{
    double x{0.0};           // 中心 X
    double y{0.0};           // 中心 Y
    double yaw{0.0};         // 朝向
    double half_length{1.0}; // 半长
    double half_width{0.5};  // 半宽
};

struct Environment
{
    Rectangle boundary{};                         // 场地边界
    Rectangle target_slot{};                      // 目标车位
    std::vector<Rectangle> static_obstacles{};    // 静态障碍物（墙体、立柱等）
};

} // namespace test_loop
