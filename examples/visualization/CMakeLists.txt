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
