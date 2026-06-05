#pragma once

#include <test_loop/sensor_hal.hpp>
#include <box2d/box2d.h>
#include <random>

namespace test_loop::sensor {

/**
 * @brief 基于 Box2D Ray-cast 的 2D LiDAR 实现
 *
 * 在 b2World 中从车辆原点向四周发射射线，检测障碍物距离。
 * 支持对距离和位姿注入高斯噪声。
 */
class RaycastLidar : public ISensorHal {
public:
    explicit RaycastLidar(b2World* world, uint32_t seed = 42);

    SensorData generate(const PhysicalState& true_state, const Environment& env) override;
    const char* name() const override { return "RaycastLidar"; }

    void set_range_noise(double std);
    void set_pose_noise(double std_x, double std_y, double std_yaw);

private:
    b2World* world_;
    std::mt19937 rng_;
    std::normal_distribution<double> range_noise_{0.0, 0.0};
    std::normal_distribution<double> pose_x_noise_{0.0, 0.0};
    std::normal_distribution<double> pose_y_noise_{0.0, 0.0};
    std::normal_distribution<double> pose_yaw_noise_{0.0, 0.0};
};

} // namespace test_loop::sensor
