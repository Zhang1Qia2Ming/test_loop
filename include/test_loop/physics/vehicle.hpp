#pragma once

#include <box2d/box2d.h>
#include <test_loop/sim_types.hpp>

namespace test_loop::physics {

/**
 * @brief 车辆物理实体
 *
 * 使用单个矩形 Dynamic Body 代表车辆底盘，通过 ApplyForceToCenter 实现纵向驱动，
 * 通过 SetAngularVelocity 施加阿克曼转向约束。
 * 适用于低速泊车场景的简化物理模型。
 */
class Vehicle {
public:
    Vehicle() = default;
    ~Vehicle() = default;

    bool spawn(b2World* world, const VehicleConfig& config, const Pose2D& initial_pose);

    void apply_control(const ControlCommand& cmd, double dt);

    PhysicalState get_state() const;
    void set_state(const PhysicalState& state);

    bool has_contact() const;

    b2Body* body() { return body_; }
    const b2Body* body() const { return body_; }

private:
    b2World* world_ = nullptr;
    b2Body* body_ = nullptr;
    VehicleConfig config_;
    double mass_ = 0.0;
    double inertia_ = 0.0;
};

} // namespace test_loop::physics
