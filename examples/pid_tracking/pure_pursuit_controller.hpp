#pragma once

#include <test_loop/i_parking_algorithm.hpp>
#include <test_loop/sim_types.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

namespace test_loop {

struct TrajectoryPoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double curvature{0.0};
};

class PurePursuitController : public IParkingAlgorithm {
public:
    void set_trajectory(const std::vector<TrajectoryPoint>& traj) { trajectory_ = traj; }
    void set_target_speed(double v) { target_speed_ = v; }
    void set_lookahead_time(double t) { lookahead_time_ = t; } // 预瞄时间 (s)
    void set_pid_gains(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

    bool initialize(const VehicleConfig& vc, const Environment&) override {
        config_ = vc;
        return true;
    }

    bool tick(const SensorData&, const PhysicalState& state, ControlCommand& out) override {
        if (trajectory_.empty()) return false;

        // === 横向：Pure Pursuit ===
        size_t closest_idx = find_closest_point(state);
        last_closest_idx_ = closest_idx;

        // 动态预瞄距离：速度越快看得越远
        double ld = std::max(1.0, lookahead_time_ * state.v);
        size_t lookahead_idx = find_lookahead_point(state, closest_idx, ld);
        const auto& target = trajectory_[lookahead_idx];

        double dx = target.x - state.x;
        double dy = target.y - state.y;
        double alpha = std::atan2(dy, dx) - state.yaw;

        // 归一化 alpha 到 [-pi, pi]
        while (alpha > M_PI) alpha -= 2.0 * M_PI;
        while (alpha < -M_PI) alpha += 2.0 * M_PI;

        double delta = std::atan(2.0 * config_.wheelbase * std::sin(alpha) / ld);
        delta = std::clamp(delta, -config_.max_steer_angle, config_.max_steer_angle);
        out.steer_angle = delta;

        // === 纵向：PID 速度控制 ===
        double v_err = target_speed_ - state.v;
        v_integral_ += v_err * 0.05;
        double v_derivative = (v_err - v_prev_err_) / 0.05;
        v_prev_err_ = v_err;

        double accel_cmd = kp_ * v_err + ki_ * v_integral_ + kd_ * v_derivative;
        accel_cmd = std::clamp(accel_cmd, -config_.max_decel, config_.max_accel);

        if (accel_cmd >= 0.0) {
            out.throttle = accel_cmd / config_.max_accel;
            out.brake = 0.0;
        } else {
            out.throttle = 0.0;
            out.brake = -accel_cmd / config_.max_decel;
        }
        out.throttle = std::clamp(out.throttle, 0.0, 1.0);
        out.brake = std::clamp(out.brake, 0.0, 1.0);

        return true;
    }

    bool serialize_state(AlgorithmState&) const override { return true; }
    bool deserialize_state(const AlgorithmState&) override {
        v_integral_ = 0.0;
        v_prev_err_ = 0.0;
        last_closest_idx_ = 0;
        return true;
    }
    const char* name() const override { return "PurePursuitController"; }

    // 诊断信息
    double lateral_error(const PhysicalState& state) const {
        if (trajectory_.empty()) return 0.0;
        double best_dist = std::hypot(trajectory_[0].x - state.x, trajectory_[0].y - state.y);
        for (size_t i = 1; i < trajectory_.size(); ++i) {
            double d = std::hypot(trajectory_[i].x - state.x, trajectory_[i].y - state.y);
            if (d < best_dist) best_dist = d;
        }
        return best_dist;
    }

    double heading_error(const PhysicalState& state) const {
        if (trajectory_.empty()) return 0.0;
        size_t closest = 0;
        double best_dist = std::hypot(trajectory_[0].x - state.x, trajectory_[0].y - state.y);
        for (size_t i = 1; i < trajectory_.size(); ++i) {
            double d = std::hypot(trajectory_[i].x - state.x, trajectory_[i].y - state.y);
            if (d < best_dist) { best_dist = d; closest = i; }
        }
        double err = trajectory_[closest].yaw - state.yaw;
        while (err > M_PI) err -= 2.0 * M_PI;
        while (err < -M_PI) err += 2.0 * M_PI;
        return err;
    }

private:
    size_t find_closest_point(const PhysicalState& state) const {
        double best_dist = std::hypot(trajectory_[0].x - state.x, trajectory_[0].y - state.y);
        size_t best = 0;
        for (size_t i = 1; i < trajectory_.size(); ++i) {
            double d = std::hypot(trajectory_[i].x - state.x, trajectory_[i].y - state.y);
            if (d < best_dist) { best_dist = d; best = i; }
        }
        return best;
    }

    size_t find_lookahead_point(const PhysicalState& state, size_t closest_idx, double ld) const {
        size_t idx = closest_idx;
        for (size_t i = 0; i < trajectory_.size(); ++i) {
            size_t next = (idx + 1) % trajectory_.size();
            double dist = std::hypot(trajectory_[next].x - state.x, trajectory_[next].y - state.y);
            if (dist >= ld) return next;
            idx = next;
        }
        return (closest_idx + 5) % trajectory_.size();
    }

    std::vector<TrajectoryPoint> trajectory_;
    VehicleConfig config_;
    double target_speed_ = 2.0;
    double lookahead_time_ = 1.0;
    double kp_ = 0.8;
    double ki_ = 0.1;
    double kd_ = 0.05;

    mutable double v_integral_ = 0.0;
    mutable double v_prev_err_ = 0.0;
    mutable size_t last_closest_idx_ = 0;
};

} // namespace test_loop
