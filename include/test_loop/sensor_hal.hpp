#pragma once

#include <array>
#include <cstddef>
#include "sim_types.hpp"

namespace test_loop
{

// ========== 传感器数据结构 ==========

constexpr size_t LIDAR_MAX_BEAMS = 360;

struct LidarScan
{
    std::array<double, LIDAR_MAX_BEAMS> ranges{};
    std::array<double, LIDAR_MAX_BEAMS> intensities{};
    size_t beam_count{360};               // 实际使用的光束数量
    double angle_min{-3.14159265358979323846};
    double angle_max{3.14159265358979323846};
    double angle_increment{0.017453292519943295}; // M_PI / 180.0
    double range_min{0.1};                // 最小有效距离 (m)
    double range_max{30.0};               // 最大有效距离 (m)
};

struct NoisyPose
{
    Pose2D pose{};
    double std_x{0.0};     // X 方向标准差
    double std_y{0.0};     // Y 方向标准差
    double std_yaw{0.0};   // 航向角标准差
};

constexpr size_t MAX_OBSTACLES = 32;

struct ObstacleList
{
    std::array<Pose2D, MAX_OBSTACLES> obstacles{};
    size_t count{0};
};

struct IMUData
{
    double accel_x{0.0};   // 纵向加速度 (m/s^2)
    double accel_y{0.0};   // 横向加速度 (m/s^2)
    double yaw_rate{0.0};  // 横摆角速度 (rad/s)
    double std_accel{0.0};
    double std_yaw_rate{0.0};
};

struct SensorData
{
    LidarScan lidar{};
    NoisyPose ego_pose{};
    ObstacleList dynamic_obstacles{};
    IMUData imu{};
};

// ========== 传感器 HAL 接口 ==========

/**
 * @brief 传感器硬件抽象层接口
 *
 * 负责从物理引擎中提取真值，并注入噪声、裁剪范围，最终生成算法可用的 SensorData。
 * 具体实现将在 Phase 2 中与 Box2D Ray-cast 对接。
 */
class ISensorHal
{
public:
    virtual ~ISensorHal() = default;

    /**
     * @brief 根据当前物理真值生成一帧传感器数据
     * @param true_state 物理引擎中的真实车辆状态
     * @param env 当前场景环境（用于提取静态障碍物真值）
     * @return 聚合后的传感器数据帧
     */
    virtual SensorData generate(const PhysicalState& true_state, const Environment& env) = 0;

    /// @brief 传感器配置名称/标识
    virtual const char* name() const = 0;
};

} // namespace test_loop
