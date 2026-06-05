#include "doctest.h"
#include <test_loop/i_parking_algorithm.hpp>

using namespace test_loop;

// Mock 实现，用于验证接口设计合理性
class MockAlgorithm : public IParkingAlgorithm
{
public:
    bool initialize(const VehicleConfig&, const Environment&) override
    {
        initialized_ = true;
        return true;
    }

    bool tick(const SensorData&, const PhysicalState&, ControlCommand& out) override
    {
        out.throttle = 0.1;
        out.steer_angle = 0.2;
        tick_count_++;
        return true;
    }

    bool serialize_state(AlgorithmState& out) const override
    {
        out.valid_size = 4;
        out.buffer[0] = 0xDE;
        out.buffer[1] = 0xAD;
        out.buffer[2] = 0xBE;
        out.buffer[3] = 0xEF;
        return true;
    }

    bool deserialize_state(const AlgorithmState& state) override
    {
        return state.valid_size == 4;
    }

    const char* name() const override { return "MockAlgorithm"; }

    bool initialized_{false};
    int tick_count_{0};
};

TEST_CASE("IParkingAlgorithm mock lifecycle")
{
    MockAlgorithm algo;
    CHECK(std::string(algo.name()) == "MockAlgorithm");
    CHECK_FALSE(algo.initialized_);

    VehicleConfig vc;
    Environment env;
    CHECK(algo.initialize(vc, env));
    CHECK(algo.initialized_);

    SensorData sd;
    PhysicalState ps;
    ControlCommand cmd;
    CHECK(algo.tick(sd, ps, cmd));
    CHECK(algo.tick_count_ == 1);
    CHECK(cmd.throttle == doctest::Approx(0.1));
    CHECK(cmd.steer_angle == doctest::Approx(0.2));

    AlgorithmState state;
    CHECK(algo.serialize_state(state));
    CHECK(state.valid_size == 4);
    CHECK(state.buffer[0] == 0xDE);

    CHECK(algo.deserialize_state(state));
}

TEST_CASE("IParkingAlgorithm tick failure handling")
{
    // 验证接口允许返回 false 表示算法异常
    struct FailingAlgorithm : public IParkingAlgorithm
    {
        bool initialize(const VehicleConfig&, const Environment&) override { return true; }
        bool tick(const SensorData&, const PhysicalState&, ControlCommand&) override { return false; }
        bool serialize_state(AlgorithmState&) const override { return true; }
        bool deserialize_state(const AlgorithmState&) override { return true; }
        const char* name() const override { return "FailingAlgorithm"; }
    };

    FailingAlgorithm algo;
    SensorData sd;
    PhysicalState ps;
    ControlCommand cmd;
    CHECK_FALSE(algo.tick(sd, ps, cmd));
}
