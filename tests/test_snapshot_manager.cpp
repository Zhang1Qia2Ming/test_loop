#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <test_loop/snapshot_manager.hpp>

using namespace test_loop;

TEST_CASE("SnapshotManager initial state")
{
    SnapshotManager<10> mgr;

    CHECK(mgr.size() == 0);
    CHECK(mgr.capacity() == 10);
    CHECK(mgr.empty());
    CHECK_FALSE(mgr.full());
    CHECK_THROWS_AS(mgr.latest(), std::out_of_range);
    CHECK_THROWS_AS(mgr.get(0), std::out_of_range);
}

TEST_CASE("SnapshotManager push and latest")
{
    SnapshotManager<10> mgr;

    SnapshotState s1;
    s1.physical_state.x = 1.0;
    s1.physical_state.y = 2.0;
    mgr.push(s1);

    CHECK(mgr.size() == 1);
    CHECK_FALSE(mgr.empty());
    CHECK(mgr.latest().physical_state.x == doctest::Approx(1.0));
    CHECK(mgr.latest().physical_state.y == doctest::Approx(2.0));

    SnapshotState s2;
    s2.physical_state.x = 3.0;
    mgr.push(s2);

    CHECK(mgr.size() == 2);
    CHECK(mgr.latest().physical_state.x == doctest::Approx(3.0));
}

TEST_CASE("SnapshotManager get with offset")
{
    SnapshotManager<10> mgr;

    for (int i = 0; i < 5; ++i) {
        SnapshotState s;
        s.physical_state.x = static_cast<double>(i);
        s.physical_state.timestamp_ms = static_cast<uint64_t>(i * 100);
        mgr.push(s);
    }

    CHECK(mgr.get(0).physical_state.x == doctest::Approx(4.0));
    CHECK(mgr.get(1).physical_state.x == doctest::Approx(3.0));
    CHECK(mgr.get(2).physical_state.x == doctest::Approx(2.0));
    CHECK(mgr.get(3).physical_state.x == doctest::Approx(1.0));
    CHECK(mgr.get(4).physical_state.x == doctest::Approx(0.0));

    CHECK(mgr.get(0).physical_state.timestamp_ms == 400);
    CHECK(mgr.get(4).physical_state.timestamp_ms == 0);
}

TEST_CASE("SnapshotManager ring buffer overwrite")
{
    SnapshotManager<3> mgr;

    for (int i = 0; i < 5; ++i) {
        SnapshotState s;
        s.physical_state.x = static_cast<double>(i);
        mgr.push(s);
    }

    CHECK(mgr.size() == 3);
    CHECK(mgr.full());
    CHECK(mgr.get(0).physical_state.x == doctest::Approx(4.0));
    CHECK(mgr.get(1).physical_state.x == doctest::Approx(3.0));
    CHECK(mgr.get(2).physical_state.x == doctest::Approx(2.0));
    CHECK_THROWS_AS(mgr.get(3), std::out_of_range);
}

TEST_CASE("SnapshotManager rollback")
{
    SnapshotManager<10> mgr;

    for (int i = 0; i < 5; ++i) {
        SnapshotState s;
        s.physical_state.v = static_cast<double>(i * 10);
        mgr.push(s);
    }

    auto r0 = mgr.rollback(0);
    CHECK(r0.physical_state.v == doctest::Approx(40.0));

    auto r2 = mgr.rollback(2);
    CHECK(r2.physical_state.v == doctest::Approx(20.0));

    auto r4 = mgr.rollback(4);
    CHECK(r4.physical_state.v == doctest::Approx(0.0));

    CHECK_THROWS_AS(mgr.rollback(5), std::out_of_range);
}

TEST_CASE("SnapshotManager clear")
{
    SnapshotManager<10> mgr;

    SnapshotState s;
    s.physical_state.yaw = 0.5;
    mgr.push(s);
    mgr.push(s);

    CHECK(mgr.size() == 2);
    mgr.clear();

    CHECK(mgr.size() == 0);
    CHECK(mgr.empty());
    CHECK_THROWS_AS(mgr.latest(), std::out_of_range);
}

TEST_CASE("SnapshotManager default capacity")
{
    SnapshotManager<> mgr; // 使用默认模板参数 1000
    CHECK(mgr.capacity() == 1000);
}
