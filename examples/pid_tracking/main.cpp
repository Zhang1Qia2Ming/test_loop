#include <test_loop/simulator.hpp>
#include <test_loop/i_parking_algorithm.hpp>
#include "stanley_controller.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace test_loop;

std::vector<TrajectoryPoint> generate_l_shape_trajectory() {
    std::vector<TrajectoryPoint> traj;
    constexpr int STRAIGHT_POINTS = 40;
    constexpr int ARC_POINTS = 60;
    constexpr double R = 10.0;

    // 第一段：直线 (0,0) -> (5,0)
    for (int i = 0; i < STRAIGHT_POINTS; ++i) {
        double t = static_cast<double>(i) / (STRAIGHT_POINTS - 1);
        TrajectoryPoint p;
        p.x = 5.0 * t;
        p.y = 0.0;
        p.yaw = 0.0;
        p.curvature = 0.0;
        traj.push_back(p);
    }

    // 第二段：1/4 圆弧，圆心 (5,10)，半径 10
    // 从角度 -pi/2 到 0（逆时针）
    for (int i = 0; i < ARC_POINTS; ++i) {
        double theta = -M_PI / 2.0 + (M_PI / 2.0) * i / (ARC_POINTS - 1);
        TrajectoryPoint p;
        p.x = 5.0 + R * std::cos(theta);
        p.y = 10.0 + R * std::sin(theta);
        p.yaw = theta + M_PI / 2.0; // 切线方向
        p.curvature = 1.0 / R;
        traj.push_back(p);
    }

    // 第三段：直线 (15,10) -> (15,20)
    for (int i = 0; i < STRAIGHT_POINTS; ++i) {
        double t = static_cast<double>(i) / (STRAIGHT_POINTS - 1);
        TrajectoryPoint p;
        p.x = 15.0;
        p.y = 10.0 + 10.0 * t;
        p.yaw = M_PI / 2.0;
        p.curvature = 0.0;
        traj.push_back(p);
    }

    return traj;
}

int main() {
    constexpr double TARGET_SPEED = 2.0; // m/s
    constexpr double DT = 0.05;          // 20 Hz
    constexpr double SIM_TIME = 20.0;    // 跑 20 秒

    auto trajectory = generate_l_shape_trajectory();

    auto controller = std::make_unique<StanleyController>();
    auto* ctrl_ptr = controller.get();
    controller->set_trajectory(trajectory);
    controller->set_target_speed(TARGET_SPEED);
    controller->set_stanley_gain(2.5);
    controller->set_pid_gains(0.8, 0.1, 0.05);

    VehicleConfig vc;
    Environment env;
    env.boundary.half_length = 25.0;
    env.boundary.half_width = 15.0;

    Pose2D initial_pose{0.0, 0.0, 0.0};

    Simulator sim;
    if (!sim.initialize(vc, env, std::move(controller), initial_pose)) {
        std::cerr << "Failed to initialize simulator\n";
        return 1;
    }

    double max_lateral_err = 0.0;
    double max_speed_err = 0.0;
    double sum_lateral_err = 0.0;
    int valid_ticks = 0;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "=== Stanley L-Shape Tracking ===\n";
    std::cout << "Target Speed: " << TARGET_SPEED << " m/s | Trajectory length: " << trajectory.size() << " pts\n";
    std::cout << "---------------------------------------------------------------\n";
    std::cout << "Tick    X       Y       Yaw     V       LatErr  HeadErr SpdErr\n";
    std::cout << "---------------------------------------------------------------\n";

    int total_ticks = static_cast<int>(SIM_TIME / DT);
    for (int i = 0; i < total_ticks; ++i) {
        if (!sim.tick(DT)) {
            std::cout << "\n*** Collision at tick " << i << " ***\n";
            break;
        }

        const auto& state = sim.ego_state();
        double lat_err = ctrl_ptr->lateral_error(state);
        double head_err = ctrl_ptr->heading_error(state);
        double spd_err = TARGET_SPEED - state.v;

        max_lateral_err = std::max(max_lateral_err, lat_err);
        max_speed_err = std::max(max_speed_err, std::abs(spd_err));
        sum_lateral_err += lat_err;
        ++valid_ticks;

        if (i % 20 == 0 || i == total_ticks - 1) {
            std::cout << std::setw(4) << i
                      << "  " << std::setw(6) << state.x
                      << "  " << std::setw(6) << state.y
                      << "  " << std::setw(6) << state.yaw
                      << "  " << std::setw(6) << state.v
                      << "  " << std::setw(6) << lat_err
                      << "  " << std::setw(6) << head_err
                      << "  " << std::setw(6) << spd_err
                      << "\n";
        }
    }

    std::cout << "---------------------------------------------------------------\n";
    std::cout << "Stats over " << valid_ticks << " ticks:\n";
    std::cout << "  Max lateral error:  " << max_lateral_err << " m\n";
    std::cout << "  Avg lateral error:  " << (valid_ticks > 0 ? sum_lateral_err / valid_ticks : 0.0) << " m\n";
    std::cout << "  Max speed error:    " << max_speed_err << " m/s\n";
    std::cout << "  Final position:     (" << sim.ego_state().x << ", " << sim.ego_state().y << ")\n";

    return 0;
}
