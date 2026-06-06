#pragma once

#include <test_loop/sim_types.hpp>
#include <test_loop/i_parking_algorithm.hpp>
#include <test_loop/sensor_hal.hpp>
#include <test_loop/snapshot_manager.hpp>
#include <test_loop/physics/world.hpp>
#include <test_loop/physics/vehicle.hpp>
#include <memory>

namespace test_loop {

/**
 * @brief ParkSim-2D 主仿真器
 *
 * 负责将物理世界、传感器 HAL、被测算法与快照管理器串联成正向闭环：
 *   保存快照 → 生成感知 → 算法 Tick → 施加控制 → 物理步进 → 碰撞检测
 * 支持单步推进与历史状态回滚。
 */
class Simulator {
public:
    Simulator();
    ~Simulator();

    bool initialize(const VehicleConfig& vehicle_config,
                    const Environment& env,
                    std::unique_ptr<IParkingAlgorithm> algo,
                    const Pose2D& initial_pose = Pose2D{0.0, 0.0, 0.0});

    bool tick(double dt);

    const PhysicalState& ego_state() const { return current_state_; }
    bool collision_detected() const { return collision_; }

    bool rollback(size_t steps_back);

    /**
     * @brief 专家纠偏：将车辆瞬间传送到指定位姿
     *
     * 用于可视化场景中的鼠标拖拽重定位。传送后保留当前速度，
     * 清除碰撞标志，并通知算法反序列化最近一次快照以恢复内部状态。
     */
    void teleport(const Pose2D& pose);

    size_t current_tick() const { return tick_count_; }

    IParkingAlgorithm* algorithm() const { return algorithm_.get(); }
    const SensorData& last_sensor_data() const { return last_sensor_data_; }

private:
    physics::PhysicsWorld world_;
    physics::Vehicle vehicle_;
    std::unique_ptr<IParkingAlgorithm> algorithm_;
    std::unique_ptr<ISensorHal> sensor_hal_;
    SnapshotManager<> snapshot_mgr_;

    Environment env_;
    VehicleConfig vehicle_config_;
    SensorData last_sensor_data_{};
    PhysicalState current_state_;
    size_t tick_count_ = 0;
    bool collision_ = false;
    bool initialized_ = false;
};

} // namespace test_loop
