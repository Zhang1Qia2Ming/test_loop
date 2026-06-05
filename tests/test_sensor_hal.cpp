#include "doctest.h"
#include <test_loop/sensor_hal.hpp>

using namespace test_loop;

TEST_CASE("SensorData default construction")
{
    SensorData sd;
    CHECK(sd.lidar.beam_count == 360);
    CHECK(sd.lidar.range_max == doctest::Approx(30.0));
    CHECK(sd.lidar.range_min == doctest::Approx(0.1));
    CHECK(sd.dynamic_obstacles.count == 0);
    CHECK(sd.ego_pose.std_x == doctest::Approx(0.0));
    CHECK(sd.imu.yaw_rate == doctest::Approx(0.0));
}

TEST_CASE("LidarScan range modification")
{
    LidarScan scan;
    scan.ranges[0] = 1.5;
    scan.ranges[359] = 10.0;
    CHECK(scan.ranges[0] == doctest::Approx(1.5));
    CHECK(scan.ranges[359] == doctest::Approx(10.0));
}

TEST_CASE("ObstacleList bounds")
{
    ObstacleList list;
    CHECK(list.count == 0);
    CHECK(list.obstacles.size() == MAX_OBSTACLES);
}
