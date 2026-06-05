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

/**
 * @brief Stanley 轨迹跟踪控制器
 *
 * 结合航向误差与前轮转角反馈，对横向误差提供直接修正。
 * 比 Pure Pursuit 在弯道和变化曲率场景下更稳定。
 */
class StanleyController : public IParkingAlgorithm {
public:
    void set_trajectory(const std::vector<TrajectoryPoint>& traj) { trajectory_ = traj; }
    void set_target_speed(double v) { target_speed_ = v; }
    void set_pid_gains(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }
    void set_stanley_gain(double k) { stanley_k_ = k; }

    bool initialize(const VehicleConfig& vc, const Environment&) override {
        config_ = vc;
        return true;
    }

    bool tick(const SensorData&, const PhysicalState& state, ControlCommand& out) override {
        if (trajectory_.empty()) return false;

        // 查找最近轨迹点
        size_t closest_idx = find_closest_point(state);
        last_closest_idx_ = closest_idx;
        const auto& closest_pt = trajectory_[closest_idx];

        // === 横向：Stanley ===
        // 航向误差
        double theta_e = closest_pt.yaw - state.yaw;
        while (theta_e > M_PI) theta_e -= 2.0 * M_PI;
        while (theta_e < -M_PI) theta_e += 2.0 * M_PI;

        // 有符号横向误差（正 = 车辆在轨迹左侧，负 = 右侧）
        double dx = state.x - closest_pt.x;
        double dy = state.y - closest_pt.y;
        double cross = std::cos(closest_pt.yaw) * dy - std::sin(closest_pt.yaw) * dx;
        double e = cross;

        // 曲率前馈 + 横向误差 P 控制（比标准 Stanley 更鲁棒）
        double kappa_ff = closest_pt.curvature;
        double delta_ff = std::atan(config_.wheelbase * kappa_ff);

        // 横向误差反馈：根据车辆相对于轨迹的左右位置调整
        // e>0 车辆在轨迹左侧，需要额外左转（正 delta）
        double delta_fb = std::atan(stanley_k_ * e / (std::abs(state.v) + 0.5));

        double delta = delta_ff + theta_e + delta_fb;
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
    const char* name() const override { return "StanleyController"; }

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

    std::vector<TrajectoryPoint> trajectory_;
    VehicleConfig config_;
    double target_speed_ = 2.0;
    double stanley_k_ = 2.5;
    double kp_ = 0.8;
    double ki_ = 0.1;
    double kd_ = 0.05;

    mutable double v_integral_ = 0.0;
    mutable double v_prev_err_ = 0.0;
    mutable size_t last_closest_idx_ = 0;
};

} // namespace test_loop
