add_executable(minimal_parking main.cpp)
target_link_libraries(minimal_parking PRIVATE test_loop)

if(NOT MSVC)
    target_compile_options(minimal_parking PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
