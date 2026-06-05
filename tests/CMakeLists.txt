# tests/CMakeLists.txt

add_executable(test_snapshot_manager test_snapshot_manager.cpp)

# 链接核心库
target_link_libraries(test_snapshot_manager PRIVATE test_loop)

# 为测试开启最严格的警告并视为错误
if(NOT MSVC)
    target_compile_options(test_snapshot_manager PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# 注册为 CTest 测试
add_test(NAME test_snapshot_manager COMMAND test_snapshot_manager)
