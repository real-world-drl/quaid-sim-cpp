cmake_minimum_required(VERSION 3.25)
project(quaid_sim_cpp)

SET(CMAKE_CXX_FLAGS -pthread)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -g")
set(CMAKE_CXX_STANDARD 20)

include_directories(include/)
# requires MUJOCO_PATH to be set
if(NOT DEFINED ENV{MUJOCO_PATH})
    message(FATAL_ERROR "MUJOCO_PATH environment variable must be set to your MuJoCo install")
endif()
include_directories($ENV{MUJOCO_PATH}/include/)
link_directories($ENV{MUJOCO_PATH}/lib/)

find_package(yaml-cpp REQUIRED)

find_package(glfw3)
find_package(PahoMqttCpp REQUIRED)

add_executable(quaid_sim_cpp src/main.cpp include/quaid_controller.h src/quaid_controller.cpp include/mqtt_controller.h src/mqtt_controller.cpp include/servo_shield.h src/servo_shield.cpp include/utils.h src/utils.cpp include/settings.h src/settings.cpp)
target_link_libraries(quaid_sim_cpp libmujoco.so libglfw.so  libpaho-mqttpp3.so libpaho-mqtt3as.so  yaml-cpp)

#add_executable(quaid_setup src/setup/setup.cpp)
#target_link_libraries(quaid_setup libmujoco.so libglfw.so  libpaho-mqttpp3.so libpaho-mqtt3as.so  yaml-cpp)
add_executable(quaid_sim_headless src/headless.cpp  include/quaid_controller.h src/quaid_controller.cpp include/mqtt_controller.h src/mqtt_controller.cpp include/servo_shield.h src/servo_shield.cpp include/utils.h src/utils.cpp include/settings.h src/settings.cpp)
target_link_libraries(quaid_sim_headless libmujoco.so libpaho-mqttpp3.so libpaho-mqtt3as.so  yaml-cpp)