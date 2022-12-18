cmake_minimum_required(VERSION 3.25)
project(quaid_sim_cpp)

SET(CMAKE_CXX_FLAGS -pthread)
set(CMAKE_CXX_STANDARD 20)

include_directories(include/)
include_directories(/home/bohm/.mujoco/mujoco-2.3.1/include/)
link_directories(/home/bohm/.mujoco/mujoco-2.3.1/lib/)

find_package(glfw3 REQUIRED)

add_executable(quaid_sim_cpp src/main.cpp)
target_link_libraries(quaid_sim_cpp libmujoco.so libglfw.so)