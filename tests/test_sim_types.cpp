#include "doctest.h"
#include <test_loop/sim_types.hpp>

using namespace test_loop;

TEST_CASE("VehicleConfig default values")
{
    VehicleConfig vc;
    CHECK(vc.wheelbase == doctest::Approx(2.8));
    CHECK(vc.track_width == doctest::Approx(1.6));
    CHECK(vc.length == doctest::Approx(4.5));
    CHECK(vc.width == doctest::Approx(1.8));
    CHECK(vc.max_steer_angle == doctest::Approx(0.52));
    CHECK(vc.max_speed == doctest::Approx(10.0));
}

TEST_CASE("Rectangle and Environment construction")
{
    Environment env;
    env.boundary.x = 0.0;
    env.boundary.y = 0.0;
    env.boundary.half_length = 10.0;
    env.boundary.half_width = 5.0;

    Rectangle obs;
    obs.x = 3.0;
    obs.y = 2.0;
    obs.half_length = 1.0;
    obs.half_width = 0.5;
    env.static_obstacles.push_back(obs);

    CHECK(env.static_obstacles.size() == 1);
    CHECK(env.static_obstacles[0].x == doctest::Approx(3.0));
    CHECK(env.target_slot.yaw == doctest::Approx(0.0));
}

TEST_CASE("AlgorithmState buffer initialization")
{
    AlgorithmState algo;
    CHECK(algo.valid_size == 0);
    CHECK(algo.buffer.size() == ALGO_STATE_MAX_SIZE);
    // value-initialized array elements are zero
    CHECK(algo.buffer[0] == 0);
    CHECK(algo.buffer[ALGO_STATE_MAX_SIZE - 1] == 0);
}

TEST_CASE("ControlCommand and PhysicalState defaults")
{
    ControlCommand cmd;
    CHECK(cmd.throttle == doctest::Approx(0.0));
    CHECK(cmd.brake == doctest::Approx(0.0));
    CHECK(cmd.steer_angle == doctest::Approx(0.0));

    PhysicalState ps;
    CHECK(ps.timestamp_ms == 0);
    CHECK(ps.v == doctest::Approx(0.0));
    CHECK(ps.w == doctest::Approx(0.0));
}
