#include "test_loop/physics/world.hpp"
#include <cmath>
#include <utility>

namespace test_loop::physics {

PhysicsWorld::PhysicsWorld()
    : world_(std::make_unique<b2World>(b2Vec2{0.0f, 0.0f})) {}

PhysicsWorld::~PhysicsWorld() = default;

bool PhysicsWorld::initialize(const Environment& env) {
    auto create_box = [&](const Rectangle& rect) -> b2Body* {
        b2BodyDef bd;
        bd.type = b2_staticBody;
        bd.position.Set(static_cast<float>(rect.x), static_cast<float>(rect.y));
        bd.angle = static_cast<float>(rect.yaw);

        b2Body* body = world_->CreateBody(&bd);

        b2PolygonShape shape;
        shape.SetAsBox(
            static_cast<float>(rect.half_length),
            static_cast<float>(rect.half_width)
        );

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 0.0f;
        fd.friction = 0.3f;
        fd.restitution = 0.0f;
        body->CreateFixture(&fd);
        return body;
    };

    // 将边界转换为四面围墙
    if (env.boundary.half_length > 0.0 && env.boundary.half_width > 0.0) {
        double cx = env.boundary.x;
        double cy = env.boundary.y;
        double hl = env.boundary.half_length;
        double hw = env.boundary.half_width;
        double yaw = env.boundary.yaw;

        double c = std::cos(yaw);
        double s = std::sin(yaw);

        auto rotate = [&](double lx, double ly) {
            return std::make_pair(cx + c * lx - s * ly,
                                  cy + s * lx + c * ly);
        };

        Rectangle wall;
        wall.yaw = yaw;

        // 上墙 (中心偏移: 0, +hw)
        {
            auto [wx, wy] = rotate(0.0, hw);
            wall.x = wx; wall.y = wy;
            wall.half_length = hl; wall.half_width = 0.05;
            static_bodies_.push_back(create_box(wall));
        }
        // 下墙 (中心偏移: 0, -hw)
        {
            auto [wx, wy] = rotate(0.0, -hw);
            wall.x = wx; wall.y = wy;
            wall.half_length = hl; wall.half_width = 0.05;
            static_bodies_.push_back(create_box(wall));
        }
        // 左墙 (中心偏移: -hl, 0)
        {
            auto [wx, wy] = rotate(-hl, 0.0);
            wall.x = wx; wall.y = wy;
            wall.half_length = 0.05; wall.half_width = hw;
            static_bodies_.push_back(create_box(wall));
        }
        // 右墙 (中心偏移: +hl, 0)
        {
            auto [wx, wy] = rotate(hl, 0.0);
            wall.x = wx; wall.y = wy;
            wall.half_length = 0.05; wall.half_width = hw;
            static_bodies_.push_back(create_box(wall));
        }
    }

    for (const auto& obs : env.static_obstacles) {
        static_bodies_.push_back(create_box(obs));
    }

    return true;
}

void PhysicsWorld::step(double dt, int velocity_iterations, int position_iterations) {
    world_->Step(static_cast<float>(dt), velocity_iterations, position_iterations);
}

} // namespace test_loop::physics
