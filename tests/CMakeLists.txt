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
