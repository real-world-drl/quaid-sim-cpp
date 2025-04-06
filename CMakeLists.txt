cmake_minimum_required(VERSION 3.25)
project(quaid_sim_cpp)

SET(CMAKE_CXX_FLAGS -pthread)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -g")
set(CMAKE_CXX_STANDARD 20)

include_directories(include/)
include_directories(~/install/mujoco/mujoco-3.1.3/include/)
link_directories(~/install/mujoco/mujoco-3.1.3/lib/)

find_package(yaml-cpp REQUIRED)

find_package(glfw3 REQUIRED)

add_executable(quaid_sim_cpp src/main.cpp include/quaid_controller.h src/quaid_controller.cpp include/mqtt_controller.h src/mqtt_controller.cpp include/servo_shield.h src/servo_shield.cpp include/utils.h src/utils.cpp include/settings.h src/settings.cpp)
target_link_libraries(quaid_sim_cpp libmujoco.so libglfw.so  libpaho-mqttpp3.so libpaho-mqtt3as.so  yaml-cpp)

#add_executable(quaid_setup src/setup/setup.cpp)
#target_link_libraries(quaid_setup libmujoco.so libglfw.so  libpaho-mqttpp3.so libpaho-mqtt3as.so  yaml-cpp)
