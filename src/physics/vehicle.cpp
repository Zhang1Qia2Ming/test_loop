#include "test_loop/physics/vehicle.hpp"
#include <cmath>
#include <algorithm>

namespace test_loop::physics {

bool Vehicle::spawn(b2World* world, const VehicleConfig& config, const Pose2D& initial_pose) {
    world_ = world;
    config_ = config;

    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(static_cast<float>(initial_pose.x), static_cast<float>(initial_pose.y));
    bd.angle = static_cast<float>(initial_pose.yaw);
    bd.linearDamping = 0.1f;
    bd.angularDamping = 0.0f;

    body_ = world_->CreateBody(&bd);

    b2PolygonShape shape;
    shape.SetAsBox(
        static_cast<float>(config.length / 2.0),
        static_cast<float>(config.width / 2.0)
    );

    b2FixtureDef fd;
    fd.shape = &shape;
    fd.density = 1.0f;
    fd.friction = 0.6f;
    fd.restitution = 0.0f;

    body_->CreateFixture(&fd);
    mass_ = body_->GetMass();
    inertia_ = body_->GetInertia();

    return true;
}

void Vehicle::apply_control(const ControlCommand& cmd, double dt) {
    if (!body_) return;

    // 读取当前车身坐标系速度
    b2Vec2 local_vel = body_->GetLocalVector(body_->GetLinearVelocity());
    double v = local_vel.x;

    // === 纵向：带加速度限制的速度控制 ===
    double target_accel = cmd.throttle * config_.max_accel - cmd.brake * config_.max_decel;
    double target_v = v + target_accel * dt;
    // 限制单帧速度变化（物理合理性）
    double max_delta_v = config_.max_accel * dt;
    target_v = std::clamp(target_v, v - max_delta_v, v + max_delta_v);
    target_v = std::clamp(target_v, -config_.max_speed, config_.max_speed);

    // 保留侧向速度分量（碰撞时 Box2D 会产生），只修改纵向分量
    b2Vec2 new_local_vel = local_vel;
    new_local_vel.x = static_cast<float>(target_v);
    b2Vec2 new_world_vel = body_->GetWorldVector(new_local_vel);
    body_->SetLinearVelocity(new_world_vel);

    // === 横向：阿克曼转向约束 ===
    double steer = std::clamp(cmd.steer_angle, -config_.max_steer_angle, config_.max_steer_angle);
    double kappa = std::tan(steer) / config_.wheelbase;
    double target_w = v * kappa;
    body_->SetAngularVelocity(static_cast<float>(target_w));
}

PhysicalState Vehicle::get_state() const {
    PhysicalState state;
    if (!body_) return state;

    b2Vec2 pos = body_->GetPosition();
    b2Vec2 vel = body_->GetLinearVelocity();

    state.x = pos.x;
    state.y = pos.y;
    state.yaw = body_->GetAngle();
    state.v = b2Dot(vel, body_->GetWorldVector(b2Vec2(1.0f, 0.0f)));
    state.w = body_->GetAngularVelocity();

    return state;
}

void Vehicle::set_state(const PhysicalState& state) {
    if (!body_) return;

    body_->SetTransform(
        b2Vec2(static_cast<float>(state.x), static_cast<float>(state.y)),
        static_cast<float>(state.yaw)
    );
    body_->SetLinearVelocity(body_->GetWorldVector(b2Vec2(static_cast<float>(state.v), 0.0f)));
    body_->SetAngularVelocity(static_cast<float>(state.w));
}

bool Vehicle::has_contact() const {
    if (!body_) return false;
    for (b2ContactEdge* ce = body_->GetContactList(); ce; ce = ce->next) {
        if (ce->contact->IsTouching()) {
            return true;
        }
    }
    return false;
}

} // namespace test_loop::physics
