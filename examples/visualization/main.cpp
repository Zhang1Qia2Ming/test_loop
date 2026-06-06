#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <test_loop/simulator.hpp>
#include <test_loop/i_parking_algorithm.hpp>
#include <cmath>
#include <memory>
#include <deque>

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

static ImVec2 screen_to_world(const ImVec2& screen, const ImVec2& center, float scale) {
    return ImVec2((screen.x - center.x) / scale, -(screen.y - center.y) / scale);
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
    float bl = static_cast<float>(env.boundary.half_length) * scale;
    float bw = static_cast<float>(env.boundary.half_width) * scale;
    ImVec2 bc = world_to_screen(static_cast<float>(env.boundary.x), static_cast<float>(env.boundary.y), center, scale);
    draw->AddRect(
        ImVec2(bc.x - bl, bc.y - bw),
        ImVec2(bc.x + bl, bc.y + bw),
        IM_COL32(120, 120, 120, 255), 0.0f, 0, 2.0f
    );

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

    ImVec2 tc = world_to_screen(static_cast<float>(env.target_slot.x), static_cast<float>(env.target_slot.y), center, scale);
    float tl = static_cast<float>(env.target_slot.half_length) * scale;
    float tw = static_cast<float>(env.target_slot.half_width) * scale;
    draw->AddRect(
        ImVec2(tc.x - tl, tc.y - tw),
        ImVec2(tc.x + tl, tc.y + tw),
        IM_COL32(0, 255, 100, 255), 0.0f, 0, 2.0f
    );
}

// ========== 绘制历史轨迹 ==========
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

    auto sim = std::make_unique<Simulator>();
    {
        auto algo = std::make_unique<CircleDrive>();
        sim->initialize(vc, env, std::move(algo), Pose2D{0.0, 0.0, 0.0});
    }

    bool running = true;
    bool step_once = false;
    float time_scale = 1.0f;
    float scale = 22.0f;
    std::deque<ImVec2> history;
    int history_length = 300;
    int history_slider = 0;
    bool show_lidar = true;
    bool show_history = true;

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

        ImDrawList* draw = ImGui::GetWindowDrawList();
        ImVec2 center(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.6f);

        // 滚轮缩放
        if (ImGui::IsWindowHovered()) {
            float wheel = io.MouseWheel;
            if (wheel != 0.0f) {
                scale *= (1.0f + wheel * 0.1f);
                scale = std::max(5.0f, std::min(200.0f, scale));
            }
        }

        // 背景网格
        for (int i = -30; i <= 30; ++i) {
            ImVec2 p1 = world_to_screen(static_cast<float>(i), -25.0f, center, scale);
            ImVec2 p2 = world_to_screen(static_cast<float>(i), 25.0f, center, scale);
            draw->AddLine(p1, p2, IM_COL32(40, 40, 40, 120));
        }
        for (int i = -25; i <= 25; ++i) {
            ImVec2 p1 = world_to_screen(-30.0f, static_cast<float>(i), center, scale);
            ImVec2 p2 = world_to_screen(30.0f, static_cast<float>(i), center, scale);
            draw->AddLine(p1, p2, IM_COL32(40, 40, 40, 120));
        }

        // 物理步进
        if (running || step_once) {
            if (!sim->tick(0.016f * time_scale)) {
                running = false;
            }
            step_once = false;
        }

        // 记录历史轨迹
        if (show_history) {
            history.push_back(world_to_screen(
                static_cast<float>(sim->ego_state().x),
                static_cast<float>(sim->ego_state().y), center, scale));
            if (history.size() > static_cast<size_t>(history_length)) {
                history.pop_front();
            }
        }

        // 绘制场景
        draw_environment(draw, env, center, scale);
        if (show_history) draw_history(draw, history);
        if (show_lidar) draw_lidar(draw, sim->last_sensor_data(), sim->ego_state(), center, scale);
        draw_vehicle(draw, sim->ego_state(), vc, center, scale, sim->collision_detected());

        // 鼠标交互：悬停世界坐标 + 点击重定位
        ImVec2 mouse_pos = io.MousePos;
        ImVec2 mouse_world = screen_to_world(mouse_pos, center, scale);
        bool hover = ImGui::IsWindowHovered();

        if (hover && ImGui::IsMouseClicked(0) && !ImGui::GetIO().WantCaptureMouse) {
            // 左键点击空白处：专家纠偏，传送到点击位置，保持当前航向
            Pose2D target;
            target.x = mouse_world.x;
            target.y = mouse_world.y;
            target.yaw = sim->ego_state().yaw;
            sim->teleport(target);
            history.clear();
            running = false; // 传送后自动暂停，方便观察
        }

        // 鼠标十字准星
        if (hover) {
            draw->AddCircle(world_to_screen(mouse_world.x, mouse_world.y, center, scale), 6.0f,
                            IM_COL32(255, 255, 0, 200), 12, 2.0f);
        }

        // 左上角 HUD
        ImGui::SetCursorPos(ImVec2(15, 15));
        ImGui::BeginChild("HUD", ImVec2(320, 420), true);
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

        ImVec2 hover_world = hover ? mouse_world : ImVec2(0, 0);
        ImGui::Text("Mouse:    (%.2f, %.2f)", hover_world.x, hover_world.y);
        ImGui::Separator();

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
        ImGui::SliderFloat("Time Scale", &time_scale, 0.1f, 3.0f);
        ImGui::SliderFloat("Zoom", &scale, 5.0f, 100.0f);
        ImGui::SliderInt("Trail Length", &history_length, 10, 2000);
        ImGui::Checkbox("Show LiDAR", &show_lidar);
        ImGui::Checkbox("Show History", &show_history);

        // 历史回溯滑动条
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

        ImGui::EndChild();

        // 右下角操作提示
        ImGui::SetCursorPos(ImVec2(io.DisplaySize.x - 260, io.DisplaySize.y - 90));
        ImGui::BeginChild("Help", ImVec2(245, 80), true);
        ImGui::Text("Controls:");
        ImGui::Text("  Mouse Wheel : Zoom");
        ImGui::Text("  Left Click  : Teleport vehicle");
        ImGui::Text("  Slide Hist. : Rollback state");
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
