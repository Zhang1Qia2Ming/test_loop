#include "test_loop/simulator.hpp"
#include "test_loop/sensor/raycast_lidar.hpp"

namespace test_loop {

Simulator::Simulator() = default;
Simulator::~Simulator() = default;

bool Simulator::initialize(const VehicleConfig& vehicle_config,
                           const Environment& env,
                           std::unique_ptr<IParkingAlgorithm> algo,
                           const Pose2D& initial_pose) {
    vehicle_config_ = vehicle_config;
    env_ = env;

    if (!world_.initialize(env)) {
        return false;
    }

    if (!vehicle_.spawn(world_.native(), vehicle_config, initial_pose)) {
        return false;
    }

    algorithm_ = std::move(algo);
    if (!algorithm_->initialize(vehicle_config, env)) {
        return false;
    }

    sensor_hal_ = std::make_unique<sensor::RaycastLidar>(world_.native());

    current_state_ = vehicle_.get_state();
    tick_count_ = 0;
    collision_ = false;
    initialized_ = true;
    return true;
}

bool Simulator::tick(double dt) {
    if (!initialized_) return false;

    // 1. 保存快照
    AlgorithmState algo_state;
    algorithm_->serialize_state(algo_state);

    SnapshotState snapshot;
    snapshot.physical_state = current_state_;
    snapshot.algo_state = algo_state;
    snapshot_mgr_.push(snapshot);

    // 2. 生成传感器数据
    last_sensor_data_ = sensor_hal_->generate(current_state_, env_);

    // 3. 算法 Tick
    ControlCommand cmd;
    if (!algorithm_->tick(last_sensor_data_, current_state_, cmd)) {
        return false;
    }

    // 4. 物理更新（子步进以提高碰撞检测精度）
    constexpr int SUBSTEPS = 5;
    double sub_dt = dt / SUBSTEPS;
    for (int i = 0; i < SUBSTEPS; ++i) {
        vehicle_.apply_control(cmd, sub_dt);
        world_.step(sub_dt);
    }

    // 5. 读取新状态
    current_state_ = vehicle_.get_state();
    current_state_.timestamp_ms = static_cast<uint64_t>(tick_count_ * dt * 1000.0);
    ++tick_count_;

    // 6. 碰撞检测
    collision_ = vehicle_.has_contact();
    return !collision_;
}

bool Simulator::rollback(size_t steps_back) {
    if (snapshot_mgr_.empty() || steps_back >= snapshot_mgr_.size()) {
        return false;
    }

    auto snapshot = snapshot_mgr_.rollback(steps_back);
    current_state_ = snapshot.physical_state;
    vehicle_.set_state(current_state_);
    algorithm_->deserialize_state(snapshot.algo_state);
    collision_ = false;
    return true;
}

} // namespace test_loop
