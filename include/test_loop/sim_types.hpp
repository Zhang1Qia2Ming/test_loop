#pragma once

#include <array>
#include <cstdint>

namespace test_loop
{

struct ControlCommand
{
    double throttle{0.0};
    double brake{0.0};
    double steer_angle{0.0};
};

struct PhysicalState
{
    uint64_t timestamp_ms{0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double v{0.0};
    double w{0.0};
};

constexpr size_t ALGO_STATE_MAX_SIZE = 256;

struct AlgorithmState
{
    std::array<uint8_t, ALGO_STATE_MAX_SIZE> buffer{};
    size_t valid_size{0};

};

struct SnapshotState
{
    PhysicalState physical_state{};
    AlgorithmState algo_state{};
    ControlCommand last_control_command{};
};

} // namespace test_loop