#include "doctest.h"
#include <test_loop/physics/world.hpp>
#include <test_loop/physics/vehicle.hpp>
#include <test_loop/sensor/raycast_lidar.hpp>

using namespace test_loop;

TEST_CASE("PhysicsWorld initialization with boundary")
{
    physics::PhysicsWorld world;
    Environment env;
    env.boundary.half_length = 10.0;
    env.boundary.half_width = 5.0;
    CHECK(world.initialize(env));
}

TEST_CASE("Vehicle spawn and get state")
{
    physics::PhysicsWorld world;
    world.initialize(Environment{});

    VehicleConfig vc;
    physics::Vehicle vehicle;
    CHECK(vehicle.spawn(world.native(), vc, Pose2D{1.0, 2.0, 0.5}));

    auto state = vehicle.get_state();
    CHECK(state.x == doctest::Approx(1.0));
    CHECK(state.y == doctest::Approx(2.0));
    CHECK(state.yaw == doctest::Approx(0.5));
    CHECK_FALSE(vehicle.has_contact());
}

TEST_CASE("Vehicle movement with throttle")
{
    physics::PhysicsWorld world;
    world.initialize(Environment{});

    VehicleConfig vc;
    physics::Vehicle vehicle;
    vehicle.spawn(world.native(), vc, Pose2D{0, 0, 0});

    ControlCommand cmd;
    cmd.throttle = 1.0;

    for (int i = 0; i < 100; ++i) {
        vehicle.apply_control(cmd, 1.0 / 60.0);
        world.step(1.0 / 60.0);
    }

    auto state = vehicle.get_state();
    CHECK(state.v > 0.0);
    CHECK(state.x > 0.0);
}

TEST_CASE("Vehicle set state and restore")
{
    physics::PhysicsWorld world;
    world.initialize(Environment{});

    VehicleConfig vc;
    physics::Vehicle vehicle;
    vehicle.spawn(world.native(), vc, Pose2D{0, 0, 0});

    ControlCommand cmd;
    cmd.throttle = 1.0;
    for (int i = 0; i < 50; ++i) {
        vehicle.apply_control(cmd, 1.0 / 60.0);
        world.step(1.0 / 60.0);
    }

    auto moved = vehicle.get_state();
    CHECK(moved.x > 0.0);

    PhysicalState original;
    original.x = 0.0;
    original.y = 0.0;
    original.yaw = 0.0;
    original.v = 0.0;
    original.w = 0.0;
    vehicle.set_state(original);

    auto restored = vehicle.get_state();
    CHECK(restored.x == doctest::Approx(0.0));
    CHECK(restored.y == doctest::Approx(0.0));
    CHECK(restored.v == doctest::Approx(0.0));
}

TEST_CASE("RaycastLidar detects boundary")
{
    physics::PhysicsWorld world;
    Environment env;
    env.boundary.half_length = 10.0;
    env.boundary.half_width = 10.0;
    world.initialize(env);

    sensor::RaycastLidar lidar(world.native());
    PhysicalState state;
    state.x = 0;
    state.y = 0;
    state.yaw = 0;

    auto data = lidar.generate(state, env);
    CHECK(data.lidar.beam_count == 360);

    // 正前方应有边界 (x=10)
    size_t front_idx = data.lidar.beam_count / 2;
    CHECK(data.lidar.ranges[front_idx] < data.lidar.range_max);
    CHECK(data.lidar.ranges[front_idx] == doctest::Approx(10.0).epsilon(0.5));

    // 正后方也应有边界 (x=-10)
    CHECK(data.lidar.ranges[0] < data.lidar.range_max);
}
