# ParkSim-2D 项目超详细技术文档

> 文档路径：`doc/project_documentation.md`  
> 项目路径：`/home/zhang/test_loop`  
> 物理引擎：Box2D v2.4.1（git submodule）  
> 语言标准：C++17  
> 构建工具：CMake >= 3.14  

本文档面向需要对项目做到 100% 细节掌握的读者，按文件级别、代码级别逐层拆解，涵盖目录结构、构建配置、每个头文件/源文件的逐行语义、数据流、状态机、单元测试覆盖点以及三个示例程序的完整执行流程。

---

## 目录

1. [项目定位与设计哲学](#1-项目定位与设计哲学)
2. [顶层目录与文件清单](#2-顶层目录与文件清单)
3. [构建系统详解](#3-构建系统详解)
4. [核心数据结构](#4-核心数据结构)
5. [抽象接口层](#5-抽象接口层)
6. [SnapshotManager：环形缓冲区回溯器](#6-snapshotmanager环形缓冲区回溯器)
7. [Physics 模块](#7-physics-模块)
8. [Sensor 模块：RaycastLidar](#8-sensor-模块raycastlidar)
9. [Simulator：主仿真器](#9-simulator主仿真器)
10. [单元测试逐文件分析](#10-单元测试逐文件分析)
11. [示例程序详解](#11-示例程序详解)
12. [数据流与运行时状态机](#12-数据流与运行时状态机)
13. [设计模式总结](#13-设计模式总结)
14. [已知问题、风险与扩展建议](#14-已知问题风险与扩展建议)

---

## 1. 项目定位与设计哲学

ParkSim-2D（仓库名 `test_loop`）是一个**极简、高确定性的 2D 泊车/规划/控制算法测试框架**。它刻意避开 Gazebo / Carla 这类重型 3D 仿真的渲染负担，专注于：

- **高确定性物理闭环**：固定时间步长 + Box2D 物理。
- **微秒级状态快照**：基于定长数组的环形缓冲区，O(1) push / rollback。
- **专家在环纠偏**：支持碰撞后回滚、鼠标瞬间重定位（teleport）。
- **历史状态回溯**：算法内部状态随物理状态一起保存与恢复。

核心假设：**前端感知已生成 BEV 图作为规划器输入，本框架只做算法验证，不做视觉包装。**

---

## 2. 顶层目录与文件清单

```text
test_loop/
├── CMakeLists.txt              # 根构建配置
├── README.md                   # 项目说明（中文）
├── .gitmodules                 # Box2D submodule 声明
├── doc/
│   └── project_documentation.md # 本文档
├── include/test_loop/          # 公共头文件（8 个）
│   ├── sim_types.hpp           # 所有 POD 数据类型
│   ├── sensor_hal.hpp          # 传感器数据 + HAL 接口
│   ├── i_parking_algorithm.hpp # 被测算法 UUT 接口
│   ├── snapshot_manager.hpp    # 环形缓冲区快照管理器（模板类）
│   ├── simulator.hpp           # 主仿真器
│   ├── physics/
│   │   ├── world.hpp           # Box2D 世界封装
│   │   └── vehicle.hpp         # 车辆刚体
│   └── sensor/
│       └── raycast_lidar.hpp   # ISensorHal 的 Ray-cast 实现
├── src/                        # 核心库实现（4 个 .cpp）
│   ├── simulator.cpp
│   ├── physics/
│   │   ├── world.cpp
│   │   └── vehicle.cpp
│   └── sensor/
│       └── raycast_lidar.cpp
├── tests/                      # doctest 单元测试
│   ├── CMakeLists.txt
│   ├── doctest.h               # 单头文件测试框架
│   ├── test_main.cpp
│   ├── test_sim_types.cpp
│   ├── test_sensor_hal.cpp
│   ├── test_snapshot_manager.cpp
│   ├── test_parking_algorithm.cpp
│   ├── test_physics.cpp
│   └── test_simulator.cpp
├── examples/                   # 3 个示例程序
│   ├── minimal_parking/        # Headless 最小闭环
│   ├── pid_tracking/           # Stanley / Pure Pursuit 轨迹跟踪
│   └── visualization/          # Dear ImGui 实时可视化
└── third_party/box2d/          # Box2D v2.4.1 完整源码 submodule
    ├── include/box2d/          # Box2D 公共头
    ├── src/                    # Box2D 实现
    └── extern/                 # glad / glfw / imgui / sajson
```

---

## 3. 构建系统详解

### 3.1 根目录 `CMakeLists.txt`

文件共 39 行，功能如下：

```cmake
cmake_minimum_required(VERSION 3.14)
project(test_loop VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

- 最低 CMake 3.14，项目名 `test_loop`，版本 `0.1.0`。
- **强制 C++17**，关闭编译器扩展，保证跨平台一致性。

```cmake
# Box2D: 关闭 testbed 和 unit-test 以加速编译
set(BOX2D_BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_BUILD_TESTBED OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/box2d)
```

- Box2D 作为子目录引入。
- 显式关闭 `BOX2D_BUILD_UNIT_TESTS` 与 `BOX2D_BUILD_TESTBED`，避免编译 Box2D 自带的大量 testbed 样例和单元测试，缩短首次构建时间。

```cmake
# 核心库
add_library(${PROJECT_NAME}
    src/physics/world.cpp
    src/physics/vehicle.cpp
    src/sensor/raycast_lidar.cpp
    src/simulator.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC box2d)
```

- 构建静态库 `test_loop`，包含 4 个 `.cpp`。
- `PUBLIC` 暴露 `include/` 目录给所有链接者。
- 链接 `box2d` 目标（由 `third_party/box2d/CMakeLists.txt` 定义）。

```cmake
if(NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 示例
add_subdirectory(examples/minimal_parking)
add_subdirectory(examples/pid_tracking)
add_subdirectory(examples/visualization)

# 测试
enable_testing()
add_subdirectory(tests)
```

- 非 MSVC 开启 `-Wall -Wextra -Wpedantic`。
- 添加三个示例子目录与测试子目录。

### 3.2 `tests/CMakeLists.txt`

```cmake
add_executable(test_loop_tests
    test_main.cpp
    test_snapshot_manager.cpp
    test_sim_types.cpp
    test_sensor_hal.cpp
    test_parking_algorithm.cpp
    test_physics.cpp
    test_simulator.cpp
)

target_link_libraries(test_loop_tests PRIVATE test_loop)

if(NOT MSVC)
    target_compile_options(test_loop_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_test(NAME test_loop_tests COMMAND test_loop_tests)
```

- 测试目标 `test_loop_tests`，单文件可执行。
- 使用 `doctest.h` 单头文件框架（无需额外 CMake 配置）。
- 链接核心库 `test_loop`。
- 非 MSVC 额外加 **`-Werror`**，测试代码必须 0 警告。

### 3.3 `examples/minimal_parking/CMakeLists.txt`

```cmake
add_executable(minimal_parking main.cpp)
target_link_libraries(minimal_parking PRIVATE test_loop)

if(NOT MSVC)
    target_compile_options(minimal_parking PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
```

- 最简单的可执行目标模板，仅链接 `test_loop`。

### 3.4 `examples/pid_tracking/CMakeLists.txt`

与 `minimal_parking` 完全一致：

```cmake
add_executable(pid_tracking main.cpp)
target_link_libraries(pid_tracking PRIVATE test_loop)

if(NOT MSVC)
    target_compile_options(pid_tracking PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
```

### 3.5 `examples/visualization/CMakeLists.txt`

```cmake
add_executable(visualization main.cpp
    ${CMAKE_SOURCE_DIR}/third_party/box2d/testbed/imgui_impl_glfw.cpp
    ${CMAKE_SOURCE_DIR}/third_party/box2d/testbed/imgui_impl_opengl3.cpp
)

# 使用 Box2D 自带的第三方渲染库
add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/box2d/extern/glad ${CMAKE_BINARY_DIR}/glad)
add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/box2d/extern/glfw ${CMAKE_BINARY_DIR}/glfw)
add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/box2d/extern/imgui ${CMAKE_BINARY_DIR}/imgui)

target_link_libraries(visualization PRIVATE test_loop glfw glad imgui)
target_include_directories(visualization PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/box2d/extern/imgui
    ${CMAKE_SOURCE_DIR}/third_party/box2d/extern/glfw/include
    ${CMAKE_SOURCE_DIR}/third_party/box2d/extern/glad/include
    ${CMAKE_SOURCE_DIR}/third_party/box2d/testbed
)

if(NOT MSVC)
    target_compile_options(visualization PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(visualization PRIVATE ${CMAKE_DL_LIBS} pthread)
endif()
```

- 编译 `imgui_impl_glfw.cpp` 和 `imgui_impl_opengl3.cpp`，这两个文件来自 Box2D testbed。
- 引入 Box2D 自带的 `glad`、`glfw`、`imgui` 子目录作为依赖。
- 链接库：`test_loop glfw glad imgui`。
- Linux 下额外链接 `${CMAKE_DL_LIBS} pthread`（`dl` 动态加载器 + pthread，GLFW 需要）。

---

## 4. 核心数据结构

所有基础数据结构集中在两个文件中：`sim_types.hpp` 与 `sensor_hal.hpp`。

### 4.1 `include/test_loop/sim_types.hpp`

共 83 行，定义在 `namespace test_loop` 中。

#### 控制与状态

```cpp
struct ControlCommand
{
    double throttle{0.0};      // 油门开度 [0, 1]
    double brake{0.0};         // 制动强度 [0, 1]
    double steer_angle{0.0};   // 前轮转角 (rad)，左正右负
};
```

- `throttle` 与 `brake` 互斥使用（算法输出逻辑里 `accel_cmd >= 0` 给油门，否则给刹车）。
- `steer_angle` 正方向约定：**左转为正**，符合车辆动力学常见约定。

```cpp
struct PhysicalState
{
    uint64_t timestamp_ms{0};  // 时间戳
    double x{0.0};             // 全局 X (m)
    double y{0.0};             // 全局 Y (m)
    double yaw{0.0};           // 航向角 (rad)
    double v{0.0};             // 纵向速度 (m/s)
    double w{0.0};             // 横摆角速度 (rad/s)
};
```

- 完全 POD，默认聚合初始化全 0。
- `timestamp_ms` 由 `Simulator::tick` 在运行时填入：`tick_count * dt * 1000`。

```cpp
constexpr size_t ALGO_STATE_MAX_SIZE = 256;

struct AlgorithmState
{
    std::array<uint8_t, ALGO_STATE_MAX_SIZE> buffer{};
    size_t valid_size{0};      // 实际有效字节数，必须 <= ALGO_STATE_MAX_SIZE
};
```

- **零动态分配**：使用 256 字节定长缓冲区。
- 算法实现者自行决定序列化格式（memcpy POD、protobuf nano、自定义等）。
- `valid_size` 是元数据，表示实际使用了多少字节。

```cpp
struct SnapshotState
{
    PhysicalState physical_state{};
    AlgorithmState algo_state{};
    ControlCommand last_control_command{};
};
```

- 快照 = 物理真值 + 算法私有状态 + 上一帧控制量。
- 注意：`last_control_command` 目前只在结构里预留，Simulator 保存快照时**并未填充**它（`SnapshotState` 默认构造后 `last_control_command` 全 0）。

#### 几何与场景

```cpp
struct Pose2D
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};
```

- 简单的 2D 位姿，yaw 正方向逆时针。

```cpp
struct VehicleConfig
{
    double wheelbase{2.8};        // 轴距 (m)
    double track_width{1.6};      // 轮距 (m)
    double length{4.5};           // 车长 (m)
    double width{1.8};            // 车宽 (m)
    double front_overhang{0.9};   // 前悬 (m)
    double rear_overhang{0.8};    // 后悬 (m)
    double max_steer_angle{0.52}; // 最大前轮转角 (rad)，约 30°
    double max_speed{10.0};       // 最大车速 (m/s)
    double max_accel{3.0};        // 最大加速度 (m/s^2)
    double max_decel{5.0};        // 最大减速度 (m/s^2)
};
```

- `track_width`、`front_overhang`、`rear_overhang` 当前**未被物理层使用**，仅作为元数据存在。
- `wheelbase`、`max_steer_angle`、`max_speed`、`max_accel`、`max_decel` 被车辆控制逻辑使用。
- `length`、`width` 直接决定 Box2D 中车辆底盘的矩形尺寸。

```cpp
struct Rectangle
{
    double x{0.0};           // 中心 X
    double y{0.0};           // 中心 Y
    double yaw{0.0};         // 朝向
    double half_length{1.0}; // 半长
    double half_width{0.5};  // 半宽
};
```

- 使用**半长/半宽**而非全尺寸，与 Box2D 的 `SetAsBox(hx, hy)` 直接对应。

```cpp
struct Environment
{
    Rectangle boundary{};                         // 场地边界
    Rectangle target_slot{};                      // 目标车位
    std::vector<Rectangle> static_obstacles{};    // 静态障碍物（墙体、立柱等）
};
```

- `boundary` 为正矩形时，`PhysicsWorld::initialize` 会将其转换成四面围墙。
- `target_slot` 当前**不参与物理碰撞**，仅在可视化示例中绘制。
- `static_obstacles` 既参与物理碰撞，也参与可视化。

### 4.2 `include/test_loop/sensor_hal.hpp`

共 84 行，定义传感器数据结构 + HAL 接口。

```cpp
constexpr size_t LIDAR_MAX_BEAMS = 360;

struct LidarScan
{
    std::array<double, LIDAR_MAX_BEAMS> ranges{};
    std::array<double, LIDAR_MAX_BEAMS> intensities{};
    size_t beam_count{360};               // 实际使用的光束数量
    double angle_min{-3.14159265358979323846};
    double angle_max{3.14159265358979323846};
    double angle_increment{0.017453292519943295}; // M_PI / 180.0
    double range_min{0.1};                // 最小有效距离 (m)
    double range_max{30.0};               // 最大有效距离 (m)
};
```

- `ranges` 与 `intensities` 都是 `std::array<double, 360>`，**intensities 当前未被填充**，始终为 0。
- `beam_count` 默认 360，但实现固定按 360 条射线处理（见 `raycast_lidar.cpp`）。
- 角度范围 `-π ~ +π`，即 360° 一周。

```cpp
struct NoisyPose
{
    Pose2D pose{};
    double std_x{0.0};     // X 方向标准差
    double std_y{0.0};     // Y 方向标准差
    double std_yaw{0.0};   // 航向角标准差
};
```

- `std_*` 字段表示该观测的噪声水平（元数据），当前实现中只是默认值 0，算法可读取作为信任度参考。

```cpp
constexpr size_t MAX_OBSTACLES = 32;

struct ObstacleList
{
    std::array<Pose2D, MAX_OBSTACLES> obstacles{};
    size_t count{0};
};
```

- 预留的动态障碍物列表。
- **当前 `RaycastLidar::generate` 未填充 `dynamic_obstacles`**，count 始终为 0。

```cpp
struct IMUData
{
    double accel_x{0.0};   // 纵向加速度 (m/s^2)
    double accel_y{0.0};   // 横向加速度 (m/s^2)
    double yaw_rate{0.0};  // 横摆角速度 (rad/s)
    double std_accel{0.0};
    double std_yaw_rate{0.0};
};
```

- `accel_x`、`accel_y` 当前**未被填充**，始终为 0。
- `yaw_rate` 被直接赋值为 `true_state.w`。

```cpp
struct SensorData
{
    LidarScan lidar{};
    NoisyPose ego_pose{};
    ObstacleList dynamic_obstacles{};
    IMUData imu{};
};
```

- 聚合结构，默认全零初始化。

#### ISensorHal 接口

```cpp
class ISensorHal
{
public:
    virtual ~ISensorHal() = default;

    virtual SensorData generate(const PhysicalState& true_state, const Environment& env) = 0;

    virtual const char* name() const = 0;
};
```

- 纯虚接口，解耦算法与物理引擎。
- `generate` 输入当前物理真值 + 环境定义，输出算法可用的 `SensorData`。
- 目前仅有一个实现 `RaycastLidar`。

---

## 5. 抽象接口层

### 5.1 `include/test_loop/i_parking_algorithm.hpp`

定义被测算法（UUT，Unit Under Test）必须实现的接口：

```cpp
class IParkingAlgorithm
{
public:
    virtual ~IParkingAlgorithm() = default;

    virtual bool initialize(const VehicleConfig& vehicle_config, const Environment& env) = 0;

    virtual bool tick(const SensorData& sensor_data, const PhysicalState& ego_state, ControlCommand& output) = 0;

    virtual bool serialize_state(AlgorithmState& out_state) const = 0;

    virtual bool deserialize_state(const AlgorithmState& state) = 0;

    virtual const char* name() const = 0;
};
```

#### 接口语义逐条解析

| 方法 | 调用时机 | 职责 |
|---|---|---|
| `initialize` | `Simulator::initialize` 阶段 | 算法根据车辆参数和场景环境做一次性初始化 |
| `tick` | 每帧 `Simulator::tick` | 读取传感器数据，输出控制指令。返回 `false` 表示算法异常，仿真器会冻结该帧 |
| `serialize_state` | 每帧 `tick` 之前 | 将算法内部状态写入定长 `AlgorithmState` |
| `deserialize_state` | `rollback` / `teleport` 时 | 从快照恢复算法内部状态 |
| `name` | 任意 | 用于日志标识 |

#### 设计要点

- `tick` 同时接收 `sensor_data`（带噪声的感知）和 `ego_state`（物理真值）。**接口注释明确建议算法只依赖 `sensor_data`**，`ego_state` 仅用于调试或可视化。
- `serialize_state` 是 `const` 方法，要求算法在序列化时不修改自身状态。
- `deserialize_state` 是非 const 方法，负责恢复内部状态。

---

## 6. SnapshotManager：环形缓冲区回溯器

### 6.1 文件：`include/test_loop/snapshot_manager.hpp`

这是一个**头文件-only 的模板类**，共 101 行。

```cpp
template <size_t Capacity = 1000>
class SnapshotManager
{
public:
    static_assert(Capacity > 0, "SnapshotManager capacity must be greater than 0");

    SnapshotManager() = default;
```

- 默认容量 1000。
- 编译期断言容量必须大于 0。

#### 核心成员

```cpp
private:
    std::array<SnapshotState, Capacity> buffer_{};
    size_t head_ = 0;   ///< 指向最新元素的下标
    size_t count_ = 0;  ///< 当前有效元素数量
```

- `buffer_` 是值初始化的定长数组，`SnapshotState` 内部所有 POD 也都被零初始化。
- `head_` 始终指向**最近一次 push 的元素**。
- `count_` 不超过 `Capacity`。

#### push 语义

```cpp
void push(const SnapshotState& state)
{
    head_ = (head_ + 1) % Capacity;
    buffer_[head_] = state;
    if (count_ < Capacity) {
        ++count_;
    }
}
```

- 先移动 `head_`，再赋值。
- 首次 push 时，`head_` 从 0 变为 1（不是从 0 开始存储）。这是一个不影响正确性的小细节。
- 当 `count_ == Capacity` 时，新元素覆盖最旧元素，实现标准环形缓冲区。

#### latest / get 语义

```cpp
const SnapshotState& latest() const
{
    if (empty()) {
        throw std::out_of_range("SnapshotManager is empty");
    }
    return buffer_[head_];
}

const SnapshotState& get(size_t offset_from_latest) const
{
    if (offset_from_latest >= count_) {
        throw std::out_of_range("SnapshotManager offset out of range");
    }
    const size_t idx = (head_ + Capacity - offset_from_latest) % Capacity;
    return buffer_[idx];
}
```

- `get(0)` == `latest()`。
- `get(1)` 返回次新，以此类推。
- 偏移量超范围时抛出 `std::out_of_range`。

#### rollback

```cpp
SnapshotState rollback(size_t steps_back) const
{
    return get(steps_back);
}
```

- 本质是 `get` 的别名，返回**按值拷贝**的 `SnapshotState`。

#### clear

```cpp
void clear()
{
    count_ = 0;
    head_ = 0;
}
```

- 不清除底层数组内容，只是逻辑重置。由于 `count_` 控制有效范围，旧数据不会再被访问到。

### 6.2 复杂度分析

| 操作 | 时间复杂度 | 空间复杂度 | 是否动态分配 |
|---|---|---|---|
| `push` | O(1) | O(1) | 否 |
| `latest` / `get` | O(1) | O(1) | 否 |
| `rollback` | O(1) | O(1)，返回拷贝 | 否 |
| `clear` | O(1) | O(1) | 否 |

### 6.3 容量计算

默认容量 1000，若 `dt = 0.05s`，则可保存 **50 秒** 的历史。  
每个 `SnapshotState` 大小约为：

- `PhysicalState`：8 * 6 = 48 字节
- `AlgorithmState`：256 + 8 = 264 字节
- `ControlCommand`：8 * 3 = 24 字节
- 合计约 336 字节

默认实例内存占用：`1000 * 336B ≈ 336KB`，完全在栈/静态区可接受范围。



---

## 7. Physics 模块

Physics 模块包含两个类：`PhysicsWorld`（Box2D 世界封装）和 `Vehicle`（车辆刚体）。

### 7.1 `include/test_loop/physics/world.hpp`

```cpp
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    bool initialize(const Environment& env);

    b2World* native() { return world_.get(); }
    const b2World* native() const { return world_.get(); }

    void step(double dt, int velocity_iterations = 8, int position_iterations = 3);

private:
    std::unique_ptr<b2World> world_;
    std::vector<b2Body*> static_bodies_;
};
```

- `native()` 提供对底层 `b2World` 的访问，方便 `Vehicle` 和 `RaycastLidar` 直接操作。
- `static_bodies_` 保存所有静态障碍物 body 指针，便于未来扩展（如动态增删障碍物），但当前仅在析构时随 `world_` 一起释放。

### 7.2 `src/physics/world.cpp`

共 96 行。

#### 构造函数

```cpp
PhysicsWorld::PhysicsWorld()
    : world_(std::make_unique<b2World>(b2Vec2{0.0f, 0.0f})) {}
```

- 重力设为 `(0, 0)`，因为是俯视 2D 泊车场景，不需要重力。

#### initialize 实现

内部使用 lambda `create_box`：

```cpp
auto create_box = [&](const Rectangle& rect) -> b2Body* {
    b2BodyDef bd;
    bd.type = b2_staticBody;
    bd.position.Set(static_cast<float>(rect.x), static_cast<float>(rect.y));
    bd.angle = static_cast<float>(rect.yaw);

    b2Body* body = world_->CreateBody(&bd);

    b2PolygonShape shape;
    shape.SetAsBox(
        static_cast<float>(rect.half_length),
        static_cast<float>(rect.half_width)
    );

    b2FixtureDef fd;
    fd.shape = &shape;
    fd.density = 0.0f;
    fd.friction = 0.3f;
    fd.restitution = 0.0f;
    body->CreateFixture(&fd);
    return body;
};
```

- 所有静态障碍物统一属性：密度 0、摩擦 0.3、弹性 0。
- 摩擦 0.3 是一个比较低的值，车辆与墙体碰撞后不会产生太大的切向阻力。

#### 边界生成逻辑

```cpp
if (env.boundary.half_length > 0.0 && env.boundary.half_width > 0.0) {
    double cx = env.boundary.x;
    double cy = env.boundary.y;
    double hl = env.boundary.half_length;
    double hw = env.boundary.half_width;
    double yaw = env.boundary.yaw;

    double c = std::cos(yaw);
    double s = std::sin(yaw);

    auto rotate = [&](double lx, double ly) {
        return std::make_pair(cx + c * lx - s * ly,
                              cy + s * lx + c * ly);
    };
```

- 支持任意旋转的边界矩形。
- `rotate(lx, ly)` 将局部坐标转换到世界坐标。

四面墙分别位于：

| 墙 | 局部中心偏移 | half_length | half_width |
|---|---|---|---|
| 上墙 | (0, +hw) | hl | 0.05 |
| 下墙 | (0, -hw) | hl | 0.05 |
| 左墙 | (-hl, 0) | 0.05 | hw |
| 右墙 | (+hl, 0) | 0.05 | hw |

- 墙厚固定为 0.05m（5cm）。
- 这样四面墙围成的内部空间正好是一个 `2*hl × 2*hw` 的矩形。

```cpp
for (const auto& obs : env.static_obstacles) {
    static_bodies_.push_back(create_box(obs));
}
```

- 最后遍历 `env.static_obstacles`，将所有自定义障碍物也加入世界。

#### step 实现

```cpp
void PhysicsWorld::step(double dt, int velocity_iterations, int position_iterations) {
    world_->Step(static_cast<float>(dt), velocity_iterations, position_iterations);
}
```

- 默认 8 速度迭代、3 位置迭代，Box2D 默认值。
- `dt` 从 `double` 转为 `float` 传入 Box2D。

### 7.3 `include/test_loop/physics/vehicle.hpp`

```cpp
class Vehicle {
public:
    bool spawn(b2World* world, const VehicleConfig& config, const Pose2D& initial_pose);

    void apply_control(const ControlCommand& cmd, double dt);

    PhysicalState get_state() const;
    void set_state(const PhysicalState& state);

    bool has_contact() const;

    b2Body* body() { return body_; }
    const b2Body* body() const { return body_; }

private:
    b2World* world_ = nullptr;
    b2Body* body_ = nullptr;
    VehicleConfig config_;
    double mass_ = 0.0;
    double inertia_ = 0.0;
};
```

- `mass_` 与 `inertia_` 在 `spawn` 后被记录，但当前 `apply_control` 中**未使用**，仅作为调试信息存在。

### 7.4 `src/physics/vehicle.cpp`

共 104 行。

#### spawn 实现

```cpp
bool Vehicle::spawn(b2World* world, const VehicleConfig& config, const Pose2D& initial_pose) {
    world_ = world;
    config_ = config;

    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(static_cast<float>(initial_pose.x), static_cast<float>(initial_pose.y));
    bd.angle = static_cast<float>(initial_pose.yaw);
    bd.linearDamping = 0.1f;
    bd.angularDamping = 0.0f;

    body_ = world_->CreateBody(&bd);

    b2PolygonShape shape;
    shape.SetAsBox(
        static_cast<float>(config.length / 2.0),
        static_cast<float>(config.width / 2.0)
    );

    b2FixtureDef fd;
    fd.shape = &shape;
    fd.density = 1.0f;
    fd.friction = 0.6f;
    fd.restitution = 0.0f;

    body_->CreateFixture(&fd);
    mass_ = body_->GetMass();
    inertia_ = body_->GetInertia();

    return true;
}
```

- `b2_dynamicBody` 表示受力和运动影响的刚体。
- `linearDamping = 0.1f`：线速度会自然衰减，模拟空气/滚动阻力。
- `angularDamping = 0.0f`：角速度不衰减。
- 车辆形状为矩形，半长 `length/2`，半宽 `width/2`。
- 密度 1.0、摩擦 0.6、弹性 0。
- 质量 = 面积 × 密度 = `length * width * 1.0`，对于默认 4.5×1.8 的车辆，质量约 8.1 kg。这是一个**非常不真实的质量**（真实车辆约 1500 kg），但因为控制逻辑直接设置速度而非施加力，质量对行为影响很小。

#### apply_control 实现

```cpp
void Vehicle::apply_control(const ControlCommand& cmd, double dt) {
    if (!body_) return;

    // 读取当前车身坐标系速度
    b2Vec2 local_vel = body_->GetLocalVector(body_->GetLinearVelocity());
    double v = local_vel.x;

    // === 纵向：带加速度限制的速度控制 ===
    double target_accel = cmd.throttle * config_.max_accel - cmd.brake * config_.max_decel;
    double target_v = v + target_accel * dt;
    // 限制单帧速度变化（物理合理性）
    double max_delta_v = config_.max_accel * dt;
    target_v = std::clamp(target_v, v - max_delta_v, v + max_delta_v);
    target_v = std::clamp(target_v, -config_.max_speed, config_.max_speed);

    // 保留侧向速度分量（碰撞时 Box2D 会产生），只修改纵向分量
    b2Vec2 new_local_vel = local_vel;
    new_local_vel.x = static_cast<float>(target_v);
    b2Vec2 new_world_vel = body_->GetWorldVector(new_local_vel);
    body_->SetLinearVelocity(new_world_vel);

    // === 横向：阿克曼转向约束 ===
    double steer = std::clamp(cmd.steer_angle, -config_.max_steer_angle, config_.max_steer_angle);
    double kappa = std::tan(steer) / config_.wheelbase;
    double target_w = v * kappa;
    body_->SetAngularVelocity(static_cast<float>(target_w));
}
```

##### 纵向控制详解

1. **读取车身坐标系速度**：`body_->GetLocalVector(world_vel)` 将世界速度转换到车身局部坐标。`local_vel.x` 就是纵向速度，`local_vel.y` 是横向速度。
2. **计算目标加速度**：`throttle * max_accel - brake * max_decel`。注意油门和刹车同时输入时会相互抵消。
3. **计算目标速度**：`v + target_accel * dt`。
4. **限幅 1**：单帧速度变化不超过 `max_accel * dt`，避免瞬间速度跳变。
5. **限幅 2**：全局速度限制在 `[-max_speed, max_speed]`。
6. **保留侧向速度**：`new_local_vel.y = local_vel.y` 保持不变，这在碰撞时比较重要，因为 Box2D 碰撞响应会产生横向反弹速度。
7. **设置世界速度**：`body_->GetWorldVector(local_vel)` 将局部速度转回世界坐标，然后 `SetLinearVelocity` 直接设置。

##### 横向控制详解

1. **转向角限幅**：`[-max_steer_angle, max_steer_angle]`。
2. **曲率计算**：`kappa = tan(steer) / wheelbase`，这是自行车模型的标准公式。
3. **目标角速度**：`target_w = v * kappa`。
4. **直接设置角速度**：`SetAngularVelocity`。

这种实现方式是**运动学控制**，不是动力学控制。它不基于力和力矩，而是直接设定速度，适合低速泊车场景。

#### get_state 实现

```cpp
PhysicalState Vehicle::get_state() const {
    PhysicalState state;
    if (!body_) return state;

    b2Vec2 pos = body_->GetPosition();
    b2Vec2 vel = body_->GetLinearVelocity();

    state.x = pos.x;
    state.y = pos.y;
    state.yaw = body_->GetAngle();
    state.v = b2Dot(vel, body_->GetWorldVector(b2Vec2(1.0f, 0.0f)));
    state.w = body_->GetAngularVelocity();

    return state;
}
```

- `state.v` 是通过点积计算的世界速度在车体纵轴上的投影，与 `apply_control` 中的 `v` 定义一致。
- `state.w` 直接读取角速度。

#### set_state 实现

```cpp
void Vehicle::set_state(const PhysicalState& state) {
    if (!body_) return;

    body_->SetTransform(
        b2Vec2(static_cast<float>(state.x), static_cast<float>(state.y)),
        static_cast<float>(state.yaw)
    );
    body_->SetLinearVelocity(body_->GetWorldVector(b2Vec2(static_cast<float>(state.v), 0.0f)));
    body_->SetAngularVelocity(static_cast<float>(state.w));
}
```

- `SetTransform` 瞬移车辆位置与角度。
- 线速度沿车体 x 轴设置，y 方向速度重置为 0。
- 角速度直接设置。

#### has_contact 实现

```cpp
bool Vehicle::has_contact() const {
    if (!body_) return false;
    for (b2ContactEdge* ce = body_->GetContactList(); ce; ce = ce->next) {
        if (ce->contact->IsTouching()) {
            return true;
        }
    }
    return false;
}
```

- 遍历 `b2Body` 的接触列表。
- 只要有接触处于 `IsTouching()` 状态就返回 true。
- 这是一个简化的碰撞检测，不区分碰撞对象（墙体、障碍物、其他车辆）。

---

## 8. Sensor 模块：RaycastLidar

### 8.1 `include/test_loop/sensor/raycast_lidar.hpp`

```cpp
class RaycastLidar : public ISensorHal {
public:
    explicit RaycastLidar(b2World* world, uint32_t seed = 42);

    SensorData generate(const PhysicalState& true_state, const Environment& env) override;
    const char* name() const override { return "RaycastLidar"; }

    void set_range_noise(double std);
    void set_pose_noise(double std_x, double std_y, double std_yaw);

private:
    b2World* world_;
    std::mt19937 rng_;
    std::normal_distribution<double> range_noise_{0.0, 0.0};
    std::normal_distribution<double> pose_x_noise_{0.0, 0.0};
    std::normal_distribution<double> pose_y_noise_{0.0, 0.0};
    std::normal_distribution<double> pose_yaw_noise_{0.0, 0.0};
};
```

- 默认噪声全为 0，需要调用 `set_range_noise` / `set_pose_noise` 才会注入噪声。
- RNG 使用 `std::mt19937`，默认种子 42（可复现）。

### 8.2 `src/sensor/raycast_lidar.cpp`

共 70 行。

#### LidarRayCastCallback

```cpp
class LidarRayCastCallback : public b2RayCastCallback {
public:
    float ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                        const b2Vec2&, float fraction) override {
        if (fixture->IsSensor()) {
            return -1.0f;
        }
        hit_point_ = point;
        hit_fraction_ = fraction;
        return fraction;
    }

    b2Vec2 hit_point_;
    float hit_fraction_ = -1.0f;
    bool has_hit() const { return hit_fraction_ >= 0.0f; }
};
```

- 继承 `b2RayCastCallback`，实现 `ReportFixture`。
- `fixture->IsSensor()` 为 true 时返回 -1，表示忽略该 fixture（射线穿过）。
- 返回 `fraction` 表示只接受比当前更近的命中点，Box2D 会自动处理最近的命中。
- `has_hit()` 检查是否发生过有效命中。

#### generate 实现

```cpp
SensorData RaycastLidar::generate(const PhysicalState& true_state, const Environment&) {
    SensorData data;

    // 带噪声的位姿观测
    data.ego_pose.pose.x = true_state.x + pose_x_noise_(rng_);
    data.ego_pose.pose.y = true_state.y + pose_y_noise_(rng_);
    data.ego_pose.pose.yaw = true_state.yaw + pose_yaw_noise_(rng_);

    b2Vec2 origin(static_cast<float>(true_state.x), static_cast<float>(true_state.y));

    for (size_t i = 0; i < data.lidar.beam_count; ++i) {
        double angle = true_state.yaw + data.lidar.angle_min + i * data.lidar.angle_increment;
        b2Vec2 dir(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
        b2Vec2 end = origin + data.lidar.range_max * dir;

        LidarRayCastCallback callback;
        world_->RayCast(&callback, origin, end);

        if (callback.has_hit()) {
            double range = (callback.hit_point_ - origin).Length();
            range += range_noise_(rng_);
            range = std::max(data.lidar.range_min,
                     std::min(data.lidar.range_max, range));
            data.lidar.ranges[i] = range;
        } else {
            data.lidar.ranges[i] = data.lidar.range_max;
        }
    }

    data.imu.yaw_rate = true_state.w;
    return data;
}
```

#### 关键细节

1. **射线原点使用 `true_state` 而非 `data.ego_pose`**：这意味着 LiDAR 测距基于真实位置，但算法收到的 ego_pose 是带噪声的。这在仿真中创造了**真实与观测的不一致性**，符合实际传感器特性。
2. **角度计算**：`true_state.yaw + angle_min + i * angle_increment`。由于 `angle_min = -π`，所以第 0 条射线指向车辆正后方，第 180 条射线指向正前方。
3. **未命中处理**：填 `range_max`（默认 30.0m）。
4. **噪声裁剪**：距离噪声后裁剪到 `[range_min, range_max]`。
5. **IMU**：只填充 `yaw_rate = true_state.w`，`accel_x/y` 始终为 0。
6. **动态障碍物**：`dynamic_obstacles.count` 始终为 0。

### 8.3 性能特征

- 每帧 360 次 `b2World::RayCast` 调用。
- Box2D 的 raycast 使用动态树（b2DynamicTree），时间复杂度约 `O(log n)`，n 为 fixture 数量。
- 无动态内存分配（`SensorData` 使用 `std::array`）。

---

## 9. Simulator：主仿真器

### 9.1 `include/test_loop/simulator.hpp`

```cpp
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
```

- `snapshot_mgr_` 使用默认模板参数 `SnapshotManager<1000>`。
- 默认初始位姿 `(0, 0, 0)`。

### 9.2 `src/simulator.cpp`

共 109 行。

#### initialize 实现

```cpp
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
```

- 初始化顺序：**物理世界 → 车辆 → 算法 → 传感器**。
- 一旦初始化失败，不会回滚之前已创建的资源（在测试场景下通常直接退出进程）。
- 传感器 HAL 在初始化后才创建，因此算法在 `initialize` 阶段**无法访问传感器**（符合设计）。

#### tick 实现

```cpp
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
```

##### 时序分析

每帧 `tick(dt)` 内部执行顺序：

1. **保存快照**：序列化算法状态 + 当前物理状态，推入 `SnapshotManager`。
2. **生成感知**：`RaycastLidar` 基于当前 `current_state_` 生成 `SensorData`。
3. **算法决策**：算法接收感知数据和物理真值，输出 `ControlCommand`。
4. **物理子步进**：将一帧分为 5 个子步，每步 `dt/5`。
   - 每个子步调用 `vehicle_.apply_control(cmd, sub_dt)` 和 `world_.step(sub_dt)`。
   - 子步进的目的是提高碰撞检测精度，避免高速时穿墙。
5. **状态更新**：读取新物理状态，更新时间戳，`tick_count_++`。
6. **碰撞检测**：检查车辆是否有接触，设置 `collision_`。

返回值：无碰撞返回 `true`，有碰撞返回 `false`。

##### 时间戳计算

```cpp
current_state_.timestamp_ms = static_cast<uint64_t>(tick_count_ * dt * 1000.0);
```

- `tick_count_` 在更新前表示**已经完成的 tick 数**。
- 因此第 N 次 tick 调用（从 0 开始）产生的时间戳是 `N * dt * 1000`。
- 例如 `dt=0.05s`，第 0 次 tick 时间戳 0ms，第 1 次 50ms，第 2 次 100ms。

#### rollback 实现

```cpp
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
```

- 防御性检查：空缓冲区或回退步数超过存储数量时返回 false。
- 恢复物理状态：调用 `vehicle_.set_state`。
- 恢复算法状态：调用 `algorithm_->deserialize_state`。
- 清除碰撞标志：`collision_ = false`。
- **不修改 `tick_count_`**：回滚后 `current_tick()` 仍然返回回滚前的 tick 数。这是一个值得注意的设计选择：时间戳不逆流，但物理状态回到过去。

#### teleport 实现

```cpp
void Simulator::teleport(const Pose2D& pose) {
    if (!initialized_) return;

    PhysicalState new_state = current_state_;
    new_state.x = pose.x;
    new_state.y = pose.y;
    new_state.yaw = pose.yaw;

    vehicle_.set_state(new_state);
    current_state_ = new_state;
    collision_ = false;

    // 尝试恢复算法到最近快照，保持内部状态一致性
    if (!snapshot_mgr_.empty()) {
        auto latest = snapshot_mgr_.rollback(0);
        algorithm_->deserialize_state(latest.algo_state);
    }
}
```

- 只修改位姿（x, y, yaw），**保留当前速度**（v, w）。
- 清除碰撞标志。
- 反序列化最近一次快照的算法状态，使算法内部状态与新位姿尽量一致。
- 如果快照管理器为空（例如初始化后未 tick），则不反序列化。

---


## 10. 单元测试逐文件分析

测试框架使用 **doctest**，一个单头文件的 C++ 测试框架。所有测试编译为单一可执行文件 `test_loop_tests`。

### 10.1 `tests/test_main.cpp`

```cpp
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
```

- 定义宏 `DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN`，使 `doctest.h` 自动提供 `main()` 函数。
- 所有测试文件只需要 `#include "doctest.h"` 即可，不需要重复定义该宏。

### 10.2 `tests/test_sim_types.cpp`

验证基础数据类型的默认行为。

| 测试用例 | 验证内容 |
|---|---|
| `VehicleConfig default values` | wheelbase=2.8, track_width=1.6, length=4.5, width=1.8, max_steer_angle=0.52, max_speed=10.0 |
| `Rectangle and Environment construction` | 可构造 Environment，设置 boundary，添加 static_obstacles，target_slot.yaw 默认 0 |
| `AlgorithmState buffer initialization` | valid_size=0，buffer.size()=256，首尾元素为 0 |
| `ControlCommand and PhysicalState defaults` | 所有字段默认 0 |

### 10.3 `tests/test_sensor_hal.cpp`

| 测试用例 | 验证内容 |
|---|---|
| `SensorData default construction` | beam_count=360, range_max=30.0, range_min=0.1, dynamic_obstacles.count=0, ego_pose 标准差 0, imu.yaw_rate=0 |
| `LidarScan range modification` | ranges[0] 和 ranges[359] 可读写 |
| `ObstacleList bounds` | count=0，obstacles.size()=MAX_OBSTACLES=32 |

### 10.4 `tests/test_snapshot_manager.cpp`

这是测试最详细的文件，共 122 行，覆盖 7 个测试用例：

| 测试用例 | 验证内容 |
|---|---|
| `SnapshotManager initial state` | size=0, capacity=10, empty=true, full=false, latest/get 抛 out_of_range |
| `SnapshotManager push and latest` | push 后 size 增加，latest 返回最新状态 |
| `SnapshotManager get with offset` | 5 个快照，get(0..4) 分别对应 x=4,3,2,1,0 |
| `SnapshotManager ring buffer overwrite` | 容量 3 时 push 5 次，只保留最近 3 个，get(3) 抛异常 |
| `SnapshotManager rollback` | rollback(0) 最新，rollback(4) 最旧，rollback(5) 抛异常 |
| `SnapshotManager clear` | clear 后 size=0，latest 抛异常 |
| `SnapshotManager default capacity` | 默认模板参数 Capacity=1000 |

### 10.5 `tests/test_parking_algorithm.cpp`

定义 `MockAlgorithm` 实现 `IParkingAlgorithm`，验证接口生命周期：

- `initialize` 返回 true，设置 `initialized_ = true`。
- `tick` 输出固定控制量 `throttle=0.1, steer_angle=0.2`，并计数。
- `serialize_state` 将 `0xDEADBEEF` 写入 buffer，设置 `valid_size=4`。
- `deserialize_state` 检查 `valid_size==4`。

| 测试用例 | 验证内容 |
|---|---|
| `IParkingAlgorithm mock lifecycle` | 完整生命周期：name → initialize → tick → serialize → deserialize |
| `IParkingAlgorithm tick failure handling` | `FailingAlgorithm::tick` 返回 false，验证接口允许异常退出 |

### 10.6 `tests/test_physics.cpp`

物理模块集成测试。

| 测试用例 | 验证内容 |
|---|---|
| `PhysicsWorld initialization with boundary` | 10×5 边界初始化成功 |
| `Vehicle spawn and get state` | 车辆在 (1,2) 生成，yaw=0.5，初始无接触 |
| `Vehicle movement with throttle` | 油门 1.0 持续 100 帧（dt=1/60），车辆 v>0, x>0 |
| `Vehicle set state and restore` | 车辆前进 50 帧后，set_state 回到原点，验证状态恢复 |
| `RaycastLidar detects boundary` | 20×20 边界场景，车辆在原点，前方（index 180）探测到约 10m 边界，后方（index 0）也有边界 |

注意：前方索引是 `data.lidar.beam_count / 2 = 180`，因为角度从 `-π`（后方）开始，经过 180 步到 `0`（前方）。

### 10.7 `tests/test_simulator.cpp`

定义 `SteadyAlgorithm`：固定输出 `throttle=0.5, steer_angle=0.05`。

| 测试用例 | 验证内容 |
|---|---|
| `Simulator initialization` | 20×10 边界场景初始化成功，tick=0，无碰撞 |
| `Simulator tick advances state` | 50×50 大边界，tick 10 次后 x>0，tick_count=10 |
| `Simulator collision and rollback` | 20×20 边界 + 前方 5m 处厚墙，运行直到碰撞，rollback(5) 后碰撞清除且位姿改变 |

最后一个测试是**核心功能的回归测试**，验证完整闭环：物理 → 感知 → 算法 → 控制 → 碰撞 → 回滚。

---

## 11. 示例程序详解

### 11.1 `examples/minimal_parking/`

#### 文件

- `CMakeLists.txt`：构建可执行文件 `minimal_parking`
- `main.cpp`：Headless 最小闭环示例

#### `main.cpp` 逐行分析

```cpp
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
```

- `DummyAlgorithm` 是最简单的算法实现：固定油门 0.3，固定转向 0.1 弧度。
- 无内部状态，因此 `serialize_state` 和 `deserialize_state` 都是空操作。

```cpp
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
```

#### 执行流程

1. 创建默认 `VehicleConfig`（轴距 2.8m，车长 4.5m，最大转向 0.52rad 等）。
2. 创建 40m×20m 的矩形场地。
3. 初始化 `Simulator`，车辆默认在原点。
4. 主循环 200 tick，每 tick 0.05s，总时长 10s。
5. 每 20 tick 打印一次车辆位姿。
6. 当 `tick` 返回 false（碰撞）时：
   - 打印碰撞信息。
   - 调用 `rollback(10)` 回退 10 步（0.5s）。
   - 打印回滚后的位姿。
   - `break` 退出循环。

#### 预期输出

车辆以固定油门 0.3、固定转向 0.1 弧度行驶，会沿着圆弧前进，最终撞上场地边界。碰撞发生后回退 10 个 tick。

---

### 11.2 `examples/pid_tracking/`

#### 文件

- `CMakeLists.txt`
- `main.cpp`
- `stanley_controller.hpp`
- `pure_pursuit_controller.hpp`

注意：当前 `main.cpp` 只 include 了 `stanley_controller.hpp`，`pure_pursuit_controller.hpp` 虽然存在但**未被使用**。

#### `stanley_controller.hpp` 详细分析

##### TrajectoryPoint

```cpp
struct TrajectoryPoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double curvature{0.0};
};
```

- 同样的结构体也在 `pure_pursuit_controller.hpp` 中定义。如果同时 include 两个头文件，会产生 **ODR（One Definition Rule）冲突**，导致链接错误。

##### StanleyController 类

```cpp
class StanleyController : public IParkingAlgorithm {
public:
    void set_trajectory(const std::vector<TrajectoryPoint>& traj) { trajectory_ = traj; }
    void set_target_speed(double v) { target_speed_ = v; }
    void set_pid_gains(double kp, double ki, double kd) { kp_=kp; ki_=ki; kd_=kd; }
    void set_stanley_gain(double k) { stanley_k_ = k; }
```

- 提供 trajectory、目标速度、PID 参数、Stanley 增益的配置接口。

```cpp
    bool initialize(const VehicleConfig& vc, const Environment&) override {
        config_ = vc;
        return true;
    }
```

- 保存车辆配置。

##### tick 横向控制

```cpp
bool tick(const SensorData&, const PhysicalState& state, ControlCommand& out) override {
    if (trajectory_.empty()) return false;

    size_t closest_idx = find_closest_point(state);
    last_closest_idx_ = closest_idx;
    const auto& closest_pt = trajectory_[closest_idx];

    // 航向误差
    double theta_e = closest_pt.yaw - state.yaw;
    while (theta_e > M_PI) theta_e -= 2.0 * M_PI;
    while (theta_e < -M_PI) theta_e += 2.0 * M_PI;

    // 有符号横向误差
    double dx = state.x - closest_pt.x;
    double dy = state.y - closest_pt.y;
    double cross = std::cos(closest_pt.yaw) * dy - std::sin(closest_pt.yaw) * dx;
    double e = cross;
```

- `find_closest_point` 暴力遍历所有轨迹点，找到欧氏距离最近点。
- 航向误差 `theta_e` 被归一化到 `[-π, π]`。
- 横向误差 `e` 使用叉积计算：
  - 若车辆在轨迹左侧，`e > 0`。
  - 若车辆在轨迹右侧，`e < 0`。

```cpp
    // 曲率前馈
    double kappa_ff = closest_pt.curvature;
    double delta_ff = std::atan(config_.wheelbase * kappa_ff);

    // 横向误差反馈
    double delta_fb = std::atan(stanley_k_ * e / (std::abs(state.v) + 0.5));

    double delta = delta_ff + theta_e + delta_fb;
    delta = std::clamp(delta, -config_.max_steer_angle, config_.max_steer_angle);
    out.steer_angle = delta;
```

- 曲率前馈：`atan(wheelbase * curvature)`，这是自行车模型的标准公式。
- 横向反馈：`atan(k * e / (|v| + 0.5))`，分母加 0.5 防止低速时分母过小导致震荡。
- 总转向角 = 前馈 + 航向误差 + 横向反馈，限幅后输出。

##### tick 纵向控制

```cpp
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
```

- 固定 dt=0.05s 的 PID 控制器。
- 积分项 `v_integral_` 没有抗积分饱和，长时间运行可能积分饱和。
- 加速度命令映射到 `[0,1]` 范围的 throttle/brake。

##### 诊断函数

```cpp
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
    // ... 返回最近轨迹点的航向误差
}
```

- 诊断函数暴力遍历所有轨迹点，时间复杂度 O(N)。
- 注意 `lateral_error` 返回的是**绝对距离**（非带符号误差）。

##### mutable 成员

```cpp
mutable double v_integral_ = 0.0;
mutable double v_prev_err_ = 0.0;
mutable size_t last_closest_idx_ = 0;
```

- 这三个成员声明为 `mutable`，但 `tick` 和 `deserialize_state` 都不是 const 方法，因此 `mutable` 是**多余的**。

#### `pure_pursuit_controller.hpp` 详细分析

结构与 Stanley 控制器几乎相同：

- 也定义了 `TrajectoryPoint`（ODR 风险）。
- 使用 Pure Pursuit 横向控制：
  - 动态预瞄距离 `ld = max(1.0, lookahead_time_ * v)`。
  - 查找预瞄点 `find_lookahead_point`。
  - 转向角 `delta = atan(2 * wheelbase * sin(alpha) / ld)`。
- 纵向控制与 Stanley 控制器完全相同（拷贝的代码）。
- `mutable` 成员同样多余。

#### `main.cpp` 逐行分析

```cpp
std::vector<TrajectoryPoint> generate_l_shape_trajectory() {
    std::vector<TrajectoryPoint> traj;
    constexpr int STRAIGHT_POINTS = 40;
    constexpr int ARC_POINTS = 60;
    constexpr double R = 10.0;
```

生成 L 型轨迹，由三段组成：

**第一段：直线 (0,0) → (5,0)**

```cpp
for (int i = 0; i < STRAIGHT_POINTS; ++i) {
    double t = static_cast<double>(i) / (STRAIGHT_POINTS - 1);
    TrajectoryPoint p;
    p.x = 5.0 * t;
    p.y = 0.0;
    p.yaw = 0.0;
    p.curvature = 0.0;
    traj.push_back(p);
}
```

- 40 个点，均匀分布。
- 起点 (0,0)，终点 (5,0)，yaw=0（朝向 +X）。

**第二段：1/4 圆弧，圆心 (5,10)，半径 10**

```cpp
for (int i = 0; i < ARC_POINTS; ++i) {
    double theta = -M_PI / 2.0 + (M_PI / 2.0) * i / (ARC_POINTS - 1);
    TrajectoryPoint p;
    p.x = 5.0 + R * std::cos(theta);
    p.y = 10.0 + R * std::sin(theta);
    p.yaw = theta + M_PI / 2.0; // 切线方向
    p.curvature = 1.0 / R;
    traj.push_back(p);
}
```

- theta 从 `-π/2` 变化到 `0`。
- theta=-π/2 时：x=5+10*0=5, y=10+10*(-1)=0，即第一段终点 (5,0)。
- theta=0 时：x=5+10*1=15, y=10+10*0=10，即圆弧终点 (15,10)。
- `yaw = theta + π/2`：切线方向。
  - theta=-π/2 时，yaw=0（朝 +X）。
  - theta=0 时，yaw=π/2（朝 +Y）。
- curvature = 1/R = 0.1。

**第三段：直线 (15,10) → (15,20)**

```cpp
for (int i = 0; i < STRAIGHT_POINTS; ++i) {
    double t = static_cast<double>(i) / (STRAIGHT_POINTS - 1);
    TrajectoryPoint p;
    p.x = 15.0;
    p.y = 10.0 + 10.0 * t;
    p.yaw = M_PI / 2.0;
    p.curvature = 0.0;
    traj.push_back(p);
}
```

- 40 个点，y 从 10 到 20。
- yaw=π/2（朝 +Y）。

总轨迹点数：40 + 60 + 40 = 140 个点。

##### 主循环

```cpp
constexpr double TARGET_SPEED = 2.0; // m/s
constexpr double DT = 0.05;          // 20 Hz
constexpr double SIM_TIME = 20.0;    // 跑 20 秒
```

- 仿真 20 秒，400 个 tick。

```cpp
auto controller = std::make_unique<StanleyController>();
auto* ctrl_ptr = controller.get();
controller->set_trajectory(trajectory);
controller->set_target_speed(TARGET_SPEED);
controller->set_stanley_gain(2.5);
controller->set_pid_gains(0.8, 0.1, 0.05);
```

- Stanley 增益 2.5，PID 参数 Kp=0.8, Ki=0.1, Kd=0.05。
- 保存裸指针 `ctrl_ptr` 用于后续调用诊断函数（`lateral_error`、`heading_error`）。

```cpp
VehicleConfig vc;
Environment env;
env.boundary.half_length = 25.0;
env.boundary.half_width = 15.0;

Pose2D initial_pose{0.0, 0.0, 0.0};

Simulator sim;
sim.initialize(vc, env, std::move(controller), initial_pose);
```

- 50m×30m 大场地，避免车辆跟踪轨迹时撞墙。
- 车辆从原点出发，初始航向 0。

##### 统计输出

```cpp
double max_lateral_err = 0.0;
double max_speed_err = 0.0;
double sum_lateral_err = 0.0;
int valid_ticks = 0;
```

- 最大横向误差、最大速度误差、累计横向误差、有效 tick 数。

```cpp
int total_ticks = static_cast<int>(SIM_TIME / DT); // 400
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
        // 打印表格
    }
}
```

- 每 20 tick（1 秒）打印一次状态。
- 横向误差使用 `lateral_error()`，返回的是到轨迹的最近距离（非带符号）。
- 平均横向误差 = `sum_lateral_err / valid_ticks`。

#### 运行结果预期

- 车辆应沿 L 型轨迹行驶，从 (0,0) → (5,0) → 圆弧 → (15,10) → (15,20)。
- 横向误差在直线段较小，在圆弧入弯/出弯处较大。
- 速度在启动阶段有上升过程，之后稳定在 2.0 m/s 附近。

---

### 11.3 `examples/visualization/`

#### 文件

- `CMakeLists.txt`：额外引入 glad/glfw/imgui
- `main.cpp`：358 行，完整交互式可视化程序

#### 算法：CircleDrive

```cpp
class CircleDrive : public IParkingAlgorithm {
public:
    bool initialize(const VehicleConfig&, const Environment&) override { return true; }
    bool tick(const SensorData&, const PhysicalState&, ControlCommand& out) override {
        out.throttle = 0.3;
        out.steer_angle = 0.25;
        return true;
    }
    bool serialize_state(AlgorithmState&) const override { return true; }
    bool deserialize_state(const AlgorithmState&) override { return true; }
    const char* name() const override { return "CircleDrive"; }
};
```

- 固定油门 0.3，固定转向 0.25 弧度，车辆会绕圈行驶。

#### 坐标变换

```cpp
static ImVec2 world_to_screen(float x, float y, const ImVec2& center, float scale) {
    return ImVec2(center.x + x * scale, center.y - y * scale);
}

static ImVec2 screen_to_world(const ImVec2& screen, const ImVec2& center, float scale) {
    return ImVec2((screen.x - center.x) / scale, -(screen.y - center.y) / scale);
}
```

- 世界坐标系：X 向右，Y 向上。
- 屏幕坐标系：X 向右，Y 向下。
- 因此 Y 方向需要取反。

#### 车辆绘制

```cpp
static void draw_vehicle(ImDrawList* draw, const PhysicalState& state,
                         const VehicleConfig& vc, const ImVec2& center, float scale,
                         bool collision) {
    ImVec2 c = world_to_screen(static_cast<float>(state.x), static_cast<float>(state.y), center, scale);
    float yaw = static_cast<float>(state.yaw);
    float hl = static_cast<float>(vc.length * 0.5) * scale;
    float hw = static_cast<float>(vc.width * 0.5) * scale;

    ImVec2 local[4] = {
        { hl,  hw},
        { hl, -hw},
        {-hl, -hw},
        {-hl,  hw}
    };

    ImVec2 scr[4];
    float co = std::cos(yaw), si = std::sin(yaw);
    for (int i = 0; i < 4; ++i) {
        float rx = co * local[i].x - si * local[i].y;
        float ry = si * local[i].x + co * local[i].y;
        scr[i] = ImVec2(c.x + rx, c.y - ry);
    }

    ImU32 color = collision ? IM_COL32(255, 60, 60, 255) : IM_COL32(0, 160, 255, 255);
    draw->AddQuadFilled(scr[0], scr[1], scr[2], scr[3], color);
    draw->AddQuad(scr[0], scr[1], scr[2], scr[3], IM_COL32(255, 255, 255, 255), 2.0f);

    float al = hl * 1.4f;
    float ax = c.x + al * co;
    float ay = c.y - al * si;
    draw->AddLine(c, ImVec2(ax, ay), IM_COL32(255, 255, 0, 255), 2.5f);
}
```

- 计算车辆四角在屏幕上的位置。
- 碰撞时车身变红，正常时为蓝色。
- 黄色箭头表示车辆航向。

#### LiDAR 绘制

```cpp
static void draw_lidar(ImDrawList* draw, const SensorData& sd,
                       const PhysicalState& state, const ImVec2& center, float scale) {
    ImVec2 origin = world_to_screen(static_cast<float>(state.x), static_cast<float>(state.y), center, scale);
    for (size_t i = 0; i < sd.lidar.beam_count; i += 4) {
        double angle = state.yaw + sd.lidar.angle_min + i * sd.lidar.angle_increment;
        double r = sd.lidar.ranges[i] * scale;
        float x2 = origin.x + static_cast<float>(r * std::cos(angle));
        float y2 = origin.y - static_cast<float>(r * std::sin(angle));
        draw->AddLine(origin, ImVec2(x2, y2), IM_COL32(255, 0, 0, 60), 1.0f);
    }
}
```

- 每 4 束射线绘制一条，减少 GPU 绘制负载。
- 射线颜色为半透明红色。

#### 环境绘制

```cpp
static void draw_environment(ImDrawList* draw, const Environment& env, const ImVec2& center, float scale) {
    // 边界（灰色线框）
    // 静态障碍物（深灰填充矩形）
    // 目标车位（绿色线框）
}
```

- 边界、障碍物、目标车位分别用不同颜色绘制。

#### 历史轨迹

```cpp
static void draw_history(ImDrawList* draw, const std::deque<ImVec2>& history) {
    if (history.size() < 2) return;
    size_t n = history.size();
    for (size_t i = 1; i < n; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(n);
        ImU32 color = IM_COL32(
            static_cast<int>(50 + 100 * t),
            static_cast<int>(150 + 80 * t),
            static_cast<int>(255),
            static_cast<int>(80 + 120 * t)
        );
        draw->AddLine(history[i - 1], history[i], color, 2.5f);
    }
}
```

- 历史轨迹使用渐变色：旧点偏暗，新点偏亮。
- 最多保存 `history_length` 个点（默认 300）。

#### 主循环关键逻辑

```cpp
while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::Begin("ParkSim-2D", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoBringToFrontOnFocus);
```

- 创建一个全屏 ImGui 窗口作为画布。
- 使用自定义绘制命令（`ImDrawList`）渲染所有内容。

```cpp
    // 滚轮缩放
    if (ImGui::IsWindowHovered()) {
        float wheel = io.MouseWheel;
        if (wheel != 0.0f) {
            scale *= (1.0f + wheel * 0.1f);
            scale = std::max(5.0f, std::min(200.0f, scale));
        }
    }
```

- 鼠标滚轮控制缩放，范围 5.0 ~ 200.0。

```cpp
    // 物理步进
    if (running || step_once) {
        if (!sim->tick(0.016f * time_scale)) {
            running = false;
        }
        step_once = false;
    }
```

- 每帧 dt = 0.016f × time_scale，约等于 16ms 一帧（60 FPS）。
- 碰撞时自动暂停。

```cpp
    // 鼠标交互：悬停世界坐标 + 点击重定位
    ImVec2 mouse_pos = io.MousePos;
    ImVec2 mouse_world = screen_to_world(mouse_pos, center, scale);
    bool hover = ImGui::IsWindowHovered();

    if (hover && ImGui::IsMouseClicked(0) && !ImGui::GetIO().WantCaptureMouse) {
        Pose2D target;
        target.x = mouse_world.x;
        target.y = mouse_world.y;
        target.yaw = sim->ego_state().yaw;
        sim->teleport(target);
        history.clear();
        running = false; // 传送后自动暂停
    }
```

- 左键点击空白处触发 `teleport`，车辆瞬移到鼠标位置，保持当前航向。
- 清空历史轨迹，暂停仿真。

#### HUD 面板

```cpp
ImGui::BeginChild("HUD", ImVec2(320, 420), true);
ImGui::Text("ParkSim-2D Visualizer");
ImGui::Separator();
ImGui::Text("Tick:     %zu", sim->current_tick());
ImGui::Text("Position: (%.2f, %.2f)", sim->ego_state().x, sim->ego_state().y);
ImGui::Text("Yaw:      %.3f rad", sim->ego_state().yaw);
ImGui::Text("Speed:    %.2f m/s", sim->ego_state().v);
```

- 左上角显示当前 tick、位姿、速度、碰撞状态。

```cpp
if (ImGui::Button(running ? "Pause" : "Resume", ImVec2(90, 25))) running = !running;
ImGui::SameLine();
if (ImGui::Button("Step", ImVec2(60, 25))) step_once = true;
ImGui::SameLine();
if (ImGui::Button("Reset", ImVec2(60, 25))) {
    sim = std::make_unique<Simulator>();
    auto algo = std::make_unique<CircleDrive>();
    sim->initialize(vc, env, std::move(algo), Pose2D{0.0, 0.0, 0.0});
    history.clear();
    running = true;
}
```

- Pause / Resume：切换运行状态。
- Step：单步推进。
- Reset：重新创建 Simulator 和算法，重置所有状态。

```cpp
ImGui::SliderFloat("Time Scale", &time_scale, 0.1f, 3.0f);
ImGui::SliderFloat("Zoom", &scale, 5.0f, 100.0f);
ImGui::SliderInt("Trail Length", &history_length, 10, 2000);
ImGui::Checkbox("Show LiDAR", &show_lidar);
ImGui::Checkbox("Show History", &show_history);
```

- time_scale：0.1x ~ 3.0x 速度。
- Zoom：5.0 ~ 100.0。
- Trail Length：10 ~ 2000 个历史点。
- Show LiDAR / Show History：切换显示。

#### 历史回溯滑动条

```cpp
int max_tick = static_cast<int>(sim->current_tick());
if (history_slider > max_tick) history_slider = max_tick;
ImGui::Separator();
ImGui::Text("History Rollback");
if (ImGui::SliderInt("Target Tick", &history_slider, 0, max_tick)) {
    // 拖动时不立即执行，释放后才回滚
}
if (ImGui::IsItemDeactivatedAfterEdit()) {
    size_t steps_back = sim->current_tick() - static_cast<size_t>(history_slider);
    if (sim->rollback(steps_back)) {
        history.clear();
        running = false;
    }
}
if (ImGui::Button("Live")) {
    history_slider = max_tick;
}
```

- 滑动条范围：0 到当前 tick。
- 拖动时只更新 UI，释放鼠标后才真正调用 `rollback`。
- `IsItemDeactivatedAfterEdit()` 是 ImGui 的 API，检测控件编辑完成事件。
- 回滚后清空历史轨迹，暂停仿真。

---

## 12. 数据流与运行时状态机

### 12.1 初始化时序

```
Simulator::initialize()
    │
    ├─► 保存 vehicle_config_, env_
    │
    ├─► PhysicsWorld::initialize(env)
    │    ├─► 创建 b2World(重力=0)
    │    ├─► 创建边界四墙
    │    └─► 创建 static_obstacles
    │
    ├─► Vehicle::spawn(world, config, initial_pose)
    │    ├─► 创建 b2_dynamicBody
    │    ├─► 设置线性阻尼 0.1
    │    └─► 创建矩形 fixture（密度1，摩擦0.6）
    │
    ├─► IParkingAlgorithm::initialize(config, env)
    │
    ├─► new RaycastLidar(world)
    │
    └─► current_state_ = vehicle_.get_state()
        tick_count_ = 0
        collision_ = false
        initialized_ = true
```

### 12.2 每 Tick 数据流

```
Simulator::tick(dt)
    │
    ├─► algorithm_->serialize_state(algo_state)
    │
    ├─► snapshot_mgr_.push({current_state_, algo_state, {}})
    │
    ├─► last_sensor_data_ = sensor_hal_->generate(current_state_, env_)
    │    ├─► 对 true_state 加位姿噪声 → data.ego_pose
    │    ├─► 360 次 b2World::RayCast
    │    │    ├─► 命中：range = |hit - origin| + 距离噪声，裁剪到 [range_min, range_max]
    │    │    └─► 未命中：range = range_max
    │    └─► data.imu.yaw_rate = true_state.w
    │
    ├─► algorithm_->tick(sensor_data, current_state_, cmd)
    │    └─► 算法输出 ControlCommand
    │
    ├─► 子步进 5 次
    │    循环 i=0..4:
    │        ├─► vehicle_.apply_control(cmd, dt/5)
    │        │    ├─► 计算 target_v，限幅
    │        │    ├─► SetLinearVelocity
    │        │    └─► SetAngularVelocity（阿克曼）
    │        └─► world_.step(dt/5)
    │
    ├─► current_state_ = vehicle_.get_state()
    ├─► current_state_.timestamp_ms = tick_count_ * dt * 1000
    ├─► tick_count_++
    ├─► collision_ = vehicle_.has_contact()
    │
    └─► return !collision_
```

### 12.3 回滚时序

```
Simulator::rollback(steps_back)
    │
    ├─► 检查 snapshot_mgr_ 非空且 steps_back < size
    │
    ├─► snapshot = snapshot_mgr_.rollback(steps_back)
    │
    ├─► current_state_ = snapshot.physical_state
    ├─► vehicle_.set_state(current_state_)
    ├─► algorithm_->deserialize_state(snapshot.algo_state)
    └─► collision_ = false
```

### 12.4 专家纠偏时序

```
Simulator::teleport(pose)
    │
    ├─► new_state = current_state_
    ├─► new_state.x/y/yaw = pose.x/y/yaw
    ├─► vehicle_.set_state(new_state)
    ├─► current_state_ = new_state
    ├─► collision_ = false
    └─► 若快照非空：algorithm_->deserialize_state(latest_snapshot.algo_state)
```

---

## 13. 设计模式总结

| 模式 | 应用位置 | 说明 |
|---|---|---|
| **Strategy（策略模式）** | `IParkingAlgorithm`、`ISensorHal` | 纯虚接口，允许替换不同算法和传感器实现 |
| **Facade（外观模式）** | `Simulator` | 将物理、传感器、算法、快照等子系统封装为简洁的 tick/rollback/teleport API |
| **Ring Buffer** | `SnapshotManager` | 定长环形数组，O(1) push/rollback，零动态分配 |
| **RAII** | `PhysicsWorld` | `unique_ptr<b2World>` 管理 Box2D 世界生命周期 |
| **Composition（组合）** | `Simulator` | 聚合 `PhysicsWorld`、`Vehicle`、`IParkingAlgorithm`、`ISensorHal`、`SnapshotManager` |
| **Data-Oriented / 零动态分配** | `LidarScan`、`AlgorithmState`、`SensorData` | 使用 `std::array` 定长存储，主循环无堆分配 |
| **Observer / Callback** | `LidarRayCastCallback` | 继承 `b2RayCastCallback`，实现射线命中回调 |

---

## 14. 已知问题、风险与扩展建议

### 14.1 已知问题

1. **ODR 冲突风险**
   - `TrajectoryPoint` 在 `stanley_controller.hpp` 和 `pure_pursuit_controller.hpp` 中各自定义。
   - 若同时 include 两个头文件，会导致重复定义编译错误。
   - **建议**：将 `TrajectoryPoint` 提取到公共头文件中。

2. **`mutable` 关键字多余**
   - `StanleyController` 和 `PurePursuitController` 中的 `v_integral_`、`v_prev_err_`、`last_closest_idx_` 声明为 `mutable`，但没有任何 const 方法会修改它们。
   - **建议**：移除 `mutable`。

3. **`last_control_command` 未填充**
   - `SnapshotState` 包含 `last_control_command`，但 `Simulator::tick` 保存快照时未赋值。
   - **建议**：在 push 前填充 `cmd`，以便回滚后知道上一帧控制量。

4. **LiDAR 射线起点使用真值**
   - `RaycastLidar::generate` 用 `true_state` 发射射线，但给算法的 `ego_pose` 是带噪声的。
   - 这种不一致性是故意的仿真特性，但文档中应明确说明。

5. **`dynamic_obstacles` 未填充**
   - `SensorData::dynamic_obstacles.count` 始终为 0。
   - 如果算法依赖动态障碍物，需要扩展实现。

6. **`intensities` 未填充**
   - `LidarScan::intensities` 始终为 0。

7. **PID 积分无抗饱和**
   - `StanleyController` 和 `PurePursuitController` 的速度 PID 积分项没有 clamp，长时间运行可能饱和。

8. **车辆质量不真实**
   - 默认车辆质量约 8.1 kg，但由于直接控制速度，对仿真行为影响较小。

9. **`tick_count_` 回滚时不减少**
   - `rollback` 不改变 `tick_count_`，意味着时间戳不逆流。
   - 这是设计选择，但需要文档化。

10. **Box2D testbed 的隐式依赖**
    - 根 CMake 关闭了 Box2D testbed，但 `visualization` 示例仍直接引用 `third_party/box2d/testbed/imgui_impl_*`。
    - 这意味着即使 `BOX2D_BUILD_TESTBED=OFF`，testbed 源文件仍会被单独编译。

### 14.2 扩展建议

1. **JSON 场景配置**
   - 将 `Environment`、`VehicleConfig` 序列化为 JSON，支持批量测试。

2. **Headless 批量压测**
   - 添加命令行模式，无需 GUI 即可运行数千组参数扫描。

3. **动态障碍物支持**
   - 在 `PhysicsWorld` 中添加可移动障碍物，并在 `RaycastLidar` 中填充 `dynamic_obstacles`。

4. **更多传感器模型**
   - 超声波、摄像头、IMU 加速度等。

5. **更真实的车辆动力学**
   - 从运动学模型（直接设速度）升级为动力学模型（轮胎模型、力和力矩）。

6. **轨迹库**
   - 提供常见的泊车轨迹生成器：平行泊车、垂直泊车、斜列泊车。

7. **评估指标**
   - 碰撞率、成功率、平均横向误差、最终位姿误差、能耗等。

---

## 附录：完整文件清单（不含 build / .git / box2d）

```
test_loop/
├── CMakeLists.txt
├── README.md
├── .gitmodules
├── doc/
│   └── project_documentation.md
├── include/test_loop/
│   ├── i_parking_algorithm.hpp
│   ├── sensor_hal.hpp
│   ├── sim_types.hpp
│   ├── simulator.hpp
│   ├── snapshot_manager.hpp
│   ├── physics/
│   │   ├── vehicle.hpp
│   │   └── world.hpp
│   └── sensor/
│       └── raycast_lidar.hpp
├── src/
│   ├── simulator.cpp
│   ├── physics/
│   │   ├── vehicle.cpp
│   │   └── world.cpp
│   └── sensor/
│       └── raycast_lidar.cpp
├── tests/
│   ├── CMakeLists.txt
│   ├── doctest.h
│   ├── test_main.cpp
│   ├── test_parking_algorithm.cpp
│   ├── test_physics.cpp
│   ├── test_sensor_hal.cpp
│   ├── test_sim_types.cpp
│   ├── test_simulator.cpp
│   └── test_snapshot_manager.cpp
└── examples/
    ├── minimal_parking/
    │   ├── CMakeLists.txt
    │   └── main.cpp
    ├── pid_tracking/
    │   ├── CMakeLists.txt
    │   ├── main.cpp
    │   ├── pure_pursuit_controller.hpp
    │   └── stanley_controller.hpp
    └── visualization/
        ├── CMakeLists.txt
        └── main.cpp
```

---

*文档生成时间：2026-06-10*  
*基于项目 commit 状态自动生成，涵盖所有非第三方源代码文件。*
