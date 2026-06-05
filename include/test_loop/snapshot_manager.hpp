#pragma once

#include <cstddef>
#include <array>
#include <stdexcept>
#include "sim_types.hpp"

namespace test_loop
{

/**
 * @brief 基于环形缓冲区的状态快照管理器
 *
 * 以固定容量的环形数组保存历史物理状态、算法状态及控制指令。
 * 当缓冲区满时，最旧的数据会被覆盖，保证 O(1) 的 push 与查询复杂度。
 *
 * @tparam Capacity 环形缓冲区最大容量，默认为 1000
 */
template <size_t Capacity = 1000>
class SnapshotManager
{
public:
    static_assert(Capacity > 0, "SnapshotManager capacity must be greater than 0");

    SnapshotManager() = default;

    /**
     * @brief 存入新的状态快照
     * @param state 当前帧的完整状态
     */
    void push(const SnapshotState& state)
    {
        head_ = (head_ + 1) % Capacity;
        buffer_[head_] = state;
        if (count_ < Capacity) {
            ++count_;
        }
    }

    /// @brief 当前已存储的快照数量
    size_t size() const { return count_; }

    /// @brief 最大容量
    constexpr size_t capacity() const { return Capacity; }

    /// @brief 是否为空
    bool empty() const { return count_ == 0; }

    /// @brief 是否已满
    bool full() const { return count_ == Capacity; }

    /**
     * @brief 获取最新的快照
     * @throws std::out_of_range 当管理器为空时
     */
    const SnapshotState& latest() const
    {
        if (empty()) {
            throw std::out_of_range("SnapshotManager is empty");
        }
        return buffer_[head_];
    }

    /**
     * @brief 获取历史快照
     * @param offset_from_latest 距离最新状态的偏移量，0 表示最新，1 表示次新，以此类推
     * @throws std::out_of_range 当偏移量超出当前存储范围时
     */
    const SnapshotState& get(size_t offset_from_latest) const
    {
        if (offset_from_latest >= count_) {
            throw std::out_of_range("SnapshotManager offset out of range");
        }
        const size_t idx = (head_ + Capacity - offset_from_latest) % Capacity;
        return buffer_[idx];
    }

    /**
     * @brief 回滚到指定步数前的状态
     * @param steps_back 回退步数，0 表示返回最新状态本身
     * @throws std::out_of_range 当回退步数超出当前存储范围时
     */
    SnapshotState rollback(size_t steps_back) const
    {
        return get(steps_back);
    }

    /// @brief 清空所有历史快照
    void clear()
    {
        count_ = 0;
        head_ = 0;
    }

private:
    std::array<SnapshotState, Capacity> buffer_{};
    size_t head_ = 0;   ///< 指向最新元素的下标
    size_t count_ = 0;  ///< 当前有效元素数量
};

} // namespace test_loop
