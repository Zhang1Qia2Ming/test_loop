#include "test_loop/sensor/raycast_lidar.hpp"
#include <cmath>
#include <algorithm>

namespace test_loop::sensor {

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

RaycastLidar::RaycastLidar(b2World* world, uint32_t seed)
    : world_(world), rng_(seed) {}

void RaycastLidar::set_range_noise(double std) {
    range_noise_ = std::normal_distribution<double>(0.0, std);
}

void RaycastLidar::set_pose_noise(double std_x, double std_y, double std_yaw) {
    pose_x_noise_ = std::normal_distribution<double>(0.0, std_x);
    pose_y_noise_ = std::normal_distribution<double>(0.0, std_y);
    pose_yaw_noise_ = std::normal_distribution<double>(0.0, std_yaw);
}

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

} // namespace test_loop::sensor
