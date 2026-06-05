#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <test_loop/simulator.hpp>
#include <test_loop/i_parking_algorithm.hpp>
#include <cmath>
#include <memory>

using namespace test_loop;

// ========== 简单算法：固定油门 + 固定转向，绕圈行驶 ==========
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

// ========== 坐标变换 ==========
static ImVec2 world_to_screen(float x, float y, const ImVec2& center, float scale) {
    return ImVec2(center.x + x * scale, center.y - y * scale);
}

// ========== 绘制车辆 ==========
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

    // 车头方向箭头
    float al = hl * 1.4f;
    float ax = c.x + al * co;
    float ay = c.y - al * si;
    draw->AddLine(c, ImVec2(ax, ay), IM_COL32(255, 255, 0, 255), 2.5f);
}

// ========== 绘制 LiDAR ==========
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

// ========== 绘制环境 ==========
static void draw_environment(ImDrawList* draw, const Environment& env, const ImVec2& center, float scale) {
    // 场地边界框
    float bl = static_cast<float>(env.boundary.half_length) * scale;
    float bw = static_cast<float>(env.boundary.half_width) * scale;
    ImVec2 bc = world_to_screen(static_cast<float>(env.boundary.x), static_cast<float>(env.boundary.y), center, scale);
    draw->AddRect(
        ImVec2(bc.x - bl, bc.y - bw),
        ImVec2(bc.x + bl, bc.y + bw),
        IM_COL32(120, 120, 120, 255), 0.0f, 0, 2.0f
    );

    // 静态障碍物
    for (const auto& obs : env.static_obstacles) {
        ImVec2 oc = world_to_screen(static_cast<float>(obs.x), static_cast<float>(obs.y), center, scale);
        float hl = static_cast<float>(obs.half_length) * scale;
        float hw = static_cast<float>(obs.half_width) * scale;
        draw->AddRectFilled(
            ImVec2(oc.x - hl, oc.y - hw),
            ImVec2(oc.x + hl, oc.y + hw),
            IM_COL32(100, 100, 100, 255)
        );
    }

    // 目标车位（绿色框）
    ImVec2 tc = world_to_screen(static_cast<float>(env.target_slot.x), static_cast<float>(env.target_slot.y), center, scale);
    float tl = static_cast<float>(env.target_slot.half_length) * scale;
    float tw = static_cast<float>(env.target_slot.half_width) * scale;
    draw->AddRect(
        ImVec2(tc.x - tl, tc.y - tw),
        ImVec2(tc.x + tl, tc.y + tw),
        IM_COL32(0, 255, 100, 255), 0.0f, 0, 2.0f
    );
}

// ========== 主函数 ==========
int main() {
    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 720, "ParkSim-2D Visualization", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    gladLoadGL(glfwGetProcAddress);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // 场景配置
    VehicleConfig vc;
    Environment env;
    env.boundary.half_length = 20.0;
    env.boundary.half_width = 15.0;

    Rectangle obs;
    obs.x = 8.0;  obs.y = 2.0;  obs.half_length = 0.2; obs.half_width = 3.0;
    env.static_obstacles.push_back(obs);
    obs.x = 12.0; obs.y = -2.0; obs.half_length = 0.2; obs.half_width = 3.0;
    env.static_obstacles.push_back(obs);

    env.target_slot.x = 15.0; env.target_slot.y = 5.0;
    env.target_slot.half_length = 2.5; env.target_slot.half_width = 1.5;

    // 初始化仿真器
    auto sim = std::make_unique<Simulator>();
    {
        auto algo = std::make_unique<CircleDrive>();
        sim->initialize(vc, env, std::move(algo), Pose2D{0.0, 0.0, 0.0});
    }

    bool running = true;
    bool step_once = false;
    float time_scale = 1.0f;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 全屏画布窗口
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("ParkSim-2D", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoBringToFrontOnFocus);

        ImDrawList* draw = ImGui::GetWindowDrawList();
        ImVec2 center(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.6f);
        float scale = 22.0f; // 1 米 = 22 像素

        // 背景网格
        for (int i = -25; i <= 25; ++i) {
            ImVec2 p1 = world_to_screen(static_cast<float>(i), -20.0f, center, scale);
            ImVec2 p2 = world_to_screen(static_cast<float>(i), 20.0f, center, scale);
            draw->AddLine(p1, p2, IM_COL32(40, 40, 40, 120));
        }
        for (int i = -20; i <= 20; ++i) {
            ImVec2 p1 = world_to_screen(-25.0f, static_cast<float>(i), center, scale);
            ImVec2 p2 = world_to_screen(25.0f, static_cast<float>(i), center, scale);
            draw->AddLine(p1, p2, IM_COL32(40, 40, 40, 120));
        }

        // 物理步进
        if (running || step_once) {
            if (!sim->tick(0.016f * time_scale)) {
                running = false; // 碰撞时自动暂停
            }
            step_once = false;
        }

        // 绘制场景
        draw_environment(draw, env, center, scale);
        draw_lidar(draw, sim->last_sensor_data(), sim->ego_state(), center, scale);
        draw_vehicle(draw, sim->ego_state(), vc, center, scale, sim->collision_detected());

        // 左上角 HUD
        ImGui::SetCursorPos(ImVec2(15, 15));
        ImGui::BeginChild("HUD", ImVec2(280, 220), true);
        ImGui::Text("ParkSim-2D Visualizer");
        ImGui::Separator();
        ImGui::Text("Tick:     %zu", sim->current_tick());
        ImGui::Text("Position: (%.2f, %.2f)", sim->ego_state().x, sim->ego_state().y);
        ImGui::Text("Yaw:      %.3f rad", sim->ego_state().yaw);
        ImGui::Text("Speed:    %.2f m/s", sim->ego_state().v);
        if (sim->collision_detected()) {
            ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "COLLISION DETECTED");
        } else {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "Running");
        }
        ImGui::Separator();
        if (ImGui::Button(running ? "Pause" : "Resume", ImVec2(80, 25))) running = !running;
        ImGui::SameLine();
        if (ImGui::Button("Step", ImVec2(60, 25))) step_once = true;
        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(60, 25))) {
            sim = std::make_unique<Simulator>();
            auto algo = std::make_unique<CircleDrive>();
            sim->initialize(vc, env, std::move(algo), Pose2D{0.0, 0.0, 0.0});
            running = true;
        }
        ImGui::SliderFloat("Time Scale", &time_scale, 0.1f, 3.0f);
        ImGui::EndChild();

        ImGui::End();

        // OpenGL 渲染
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.08f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // 清理
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
