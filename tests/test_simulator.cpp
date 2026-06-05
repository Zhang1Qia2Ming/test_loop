#include "doctest.h"
#include <test_loop/simulator.hpp>
#include <test_loop/i_parking_algorithm.hpp>

using namespace test_loop;

class SteadyAlgorithm : public IParkingAlgorithm {
public:
    bool initialize(const VehicleConfig&, const Environment&) override { return true; }

    bool tick(const SensorData&, const PhysicalState&, ControlCommand& out) override {
        out.throttle = 0.5;
        out.steer_angle = 0.05;
        return true;
    }

    bool serialize_state(AlgorithmState&) const override { return true; }
    bool deserialize_state(const AlgorithmState&) override { return true; }
    const char* name() const override { return "SteadyAlgorithm"; }
};

TEST_CASE("Simulator initialization")
{
    VehicleConfig vc;
    Environment env;
    env.boundary.half_length = 20.0;
    env.boundary.half_width = 10.0;

    auto algo = std::make_unique<SteadyAlgorithm>();
    Simulator sim;
    CHECK(sim.initialize(vc, env, std::move(algo)));
    CHECK(sim.current_tick() == 0);
    CHECK_FALSE(sim.collision_detected());
}

TEST_CASE("Simulator tick advances state")
{
    VehicleConfig vc;
    Environment env;
    env.boundary.half_length = 50.0;
    env.boundary.half_width = 50.0;

    auto algo = std::make_unique<SteadyAlgorithm>();
    Simulator sim;
    sim.initialize(vc, env, std::move(algo));

    auto initial = sim.ego_state();
    CHECK(initial.x == doctest::Approx(0.0));

    for (int i = 0; i < 10; ++i) {
        CHECK(sim.tick(0.05));
    }

    auto current = sim.ego_state();
    CHECK(current.x > 0.0);
    CHECK(sim.current_tick() == 10);
}

TEST_CASE("Simulator collision and rollback")
{
    VehicleConfig vc;
    Environment env;
    env.boundary.half_length = 20.0;
    env.boundary.half_width = 20.0;

    // 在正前方 5.0m 处放一堵厚墙，确保有足够的 tick 积累快照
    Rectangle wall;
    wall.x = 5.0;
    wall.y = 0.0;
    wall.half_length = 0.5;
    wall.half_width = 2.0;
    env.static_obstacles.push_back(wall);

    auto algo = std::make_unique<SteadyAlgorithm>();
    Simulator sim;
    sim.initialize(vc, env, std::move(algo));

    bool running = true;
    size_t ticks = 0;
    while (running && ticks < 1000) {
        running = sim.tick(0.05);
        ++ticks;
    }

    CHECK(sim.collision_detected());
    CHECK(ticks < 1000);
    CHECK(ticks > 5); // 确保有足够的快照可供回滚

    auto pre_rollback = sim.ego_state();
    CHECK(sim.rollback(5));
    CHECK_FALSE(sim.collision_detected());

    auto post_rollback = sim.ego_state();
    CHECK(post_rollback.x != doctest::Approx(pre_rollback.x));
}
