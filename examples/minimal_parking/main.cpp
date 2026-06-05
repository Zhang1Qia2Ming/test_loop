#include <test_loop/simulator.hpp>
#include <test_loop/i_parking_algorithm.hpp>
#include <iostream>

using namespace test_loop;

class DummyAlgorithm : public IParkingAlgorithm {
public:
    bool initialize(const VehicleConfig&, const Environment&) override { return true; }

    bool tick(const SensorData&, const PhysicalState&, ControlCommand& out) override {
        out.throttle = 0.3;
        out.steer_angle = 0.1;
        return true;
    }

    bool serialize_state(AlgorithmState&) const override { return true; }
    bool deserialize_state(const AlgorithmState&) override { return true; }
    const char* name() const override { return "DummyAlgorithm"; }
};

int main() {
    VehicleConfig vc;
    Environment env;
    env.boundary.half_length = 20.0;
    env.boundary.half_width = 10.0;

    auto algo = std::make_unique<DummyAlgorithm>();

    Simulator sim;
    if (!sim.initialize(vc, env, std::move(algo))) {
        std::cerr << "Failed to initialize simulator\n";
        return 1;
    }

    for (size_t i = 0; i < 200; ++i) {
        if (!sim.tick(0.05)) {
            std::cout << "Collision at tick " << i << "! Rolling back 10 steps...\n";
            if (sim.rollback(10)) {
                std::cout << "Rolled back to state: x=" << sim.ego_state().x
                          << " y=" << sim.ego_state().y << "\n";
            }
            break;
        }
        if (i % 20 == 0) {
            const auto& s = sim.ego_state();
            std::cout << "Tick " << i << ": x=" << s.x
                      << " y=" << s.y << " yaw=" << s.yaw
                      << " v=" << s.v << "\n";
        }
    }
    return 0;
}
