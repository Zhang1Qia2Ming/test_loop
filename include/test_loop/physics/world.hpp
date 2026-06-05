#pragma once

#include <box2d/box2d.h>
#include <test_loop/sim_types.hpp>
#include <memory>
#include <vector>

namespace test_loop::physics {

/**
 * @brief Box2D 物理世界封装
 *
 * 负责管理 b2World 生命周期、静态障碍物创建与物理步进。
 * 场景中的重力设为 0（俯视 2D 泊车视角）。
 */
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    bool initialize(const Environment& env);

    b2World* native() { return world_.get(); }
    const b2World* native() const { return world_.get(); }

    void step(double dt, int velocity_iterations = 8, int position_iterations = 3);

private:
    std::unique_ptr<b2World> world_;
    std::vector<b2Body*> static_bodies_;
};

} // namespace test_loop::physics
