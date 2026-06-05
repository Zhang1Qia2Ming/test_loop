#pragma once

#include "sim_types.hpp"
#include "sensor_hal.hpp"

namespace test_loop
{

/**
 * @brief 被测泊车算法 (UUT) 接入接口
 *
 * 所有需要接入 ParkSim-2D 的泊车/规划/控制算法均需实现此接口。
 * 通过 serialize_state / deserialize_state 支持算法状态的历史回溯。
 */
class IParkingAlgorithm
{
public:
    virtual ~IParkingAlgorithm() = default;

    /**
     * @brief 算法初始化
     * @param vehicle_config 车辆几何与动力学参数
     * @param env 当前仿真场景环境
     * @return true 初始化成功
     */
    virtual bool initialize(const VehicleConfig& vehicle_config, const Environment& env) = 0;

    /**
     * @brief 主循环入口，每 Tick 调用一次
     * @param sensor_data 传感器 HAL 输出的感知数据（含噪声）
     * @param ego_state 物理引擎中的车辆真值（供调试使用，算法原则上应只依赖 sensor_data）
     * @param output [out] 算法输出的控制指令
     * @return true 正常运行；false 算法异常，仿真器可据此冻结并触发回溯
     */
    virtual bool tick(const SensorData& sensor_data, const PhysicalState& ego_state, ControlCommand& output) = 0;

    /**
     * @brief 序列化算法内部状态
     * @param out_state [out] 定长输出缓冲区，valid_size 需由实现方填写
     * @return true 序列化成功
     */
    virtual bool serialize_state(AlgorithmState& out_state) const = 0;

    /**
     * @brief 从快照反序列化恢复算法内部状态
     * @param state 之前保存的快照
     * @return true 恢复成功
     */
    virtual bool deserialize_state(const AlgorithmState& state) = 0;

    /// @brief 算法名称，用于日志标识
    virtual const char* name() const = 0;
};

} // namespace test_loop
