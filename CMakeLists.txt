cmake_minimum_required(VERSION 3.9.6)
project(dice-simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -Wextra")
set(OpenGL_GL_PREFERENCE GLVND)

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

find_package(OpenGL REQUIRED)
find_package(glfw3 3.2 REQUIRED)

add_executable(dice-simulator src/main.cpp src/glad.c src/visualizer.cpp src/shaders/shader.cpp)

target_include_directories(dice-simulator PRIVATE "include")
target_link_libraries(dice-simulator GL X11 pthread Xrandr Xi dl glfw)

set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
set(GLFW_INSTALL OFF CACHE INTERNAL "Generate installation target")

target_compile_definitions(dice-simulator PRIVATE "GLFW_INCLUDE_NONE")

