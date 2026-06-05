add_executable(pid_tracking main.cpp)
target_link_libraries(pid_tracking PRIVATE test_loop)

if(NOT MSVC)
    target_compile_options(pid_tracking PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
