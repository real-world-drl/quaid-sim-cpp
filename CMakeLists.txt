cmake_minimum_required(VERSION 3.25)
project(quaid_sim_cpp)

set(CMAKE_CXX_STANDARD 20)

# --- Includes ---
include_directories(${CMAKE_SOURCE_DIR}/include)

# --- MuJoCo ---
set(MUJOCO_PATH "C:/Users/Mathew/Desktop/Quaid_Sim/mujoco-3.3.0")
include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/lib)

# --- Other dependencies (vcpkg) ---
find_package(glfw3 CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

# --- Fix for missing EMPTY_BIN symbol (PAHO) ---
file(WRITE ${CMAKE_BINARY_DIR}/paho_fix.cpp
"#include <mqtt/message.h>
namespace mqtt {
    const std::string message::EMPTY_BIN;
}
")

# =====================================================================
# GUI / Full Simulation Build Target
# =====================================================================
add_executable(quaid_sim_cpp
    src/main.cpp
    src/quaid_controller.cpp
    src/mqtt_controller.cpp
    src/servo_shield.cpp
    src/utils.cpp
    src/settings.cpp
    ${CMAKE_BINARY_DIR}/paho_fix.cpp
)

target_include_directories(quaid_sim_cpp PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${VCPKG_INSTALLED_DIR}/x64-windows/include
)

# --- Link libraries (Windows .lib format) ---
link_directories(${VCPKG_INSTALLED_DIR}/x64-windows/lib)

target_link_libraries(quaid_sim_cpp
    # MuJoCo
    "${MUJOCO_PATH}/lib/mujoco.lib"

    # vcpkg-managed libs
    "${VCPKG_INSTALLED_DIR}/x64-windows/lib/glfw3dll.lib"
    "${VCPKG_INSTALLED_DIR}/x64-windows/lib/yaml-cpp.lib"

    # MQTT: C++ wrapper and async backend
    "${VCPKG_INSTALLED_DIR}/x64-windows/lib/paho-mqttpp3.lib"
    "${VCPKG_INSTALLED_DIR}/x64-windows/lib/paho-mqtt3as.lib"
)

# --- Definitions ---
add_compile_definitions(uint=unsigned\ int)
target_compile_definitions(quaid_sim_cpp PRIVATE YAML_CPP_STATIC_DEFINE)

# =====================================================================
# Headless Build Target
# =====================================================================
add_executable(quaid_sim_headless
    src/headless.cpp

    # Shared sources
    src/quaid_controller.cpp
    src/mqtt_controller.cpp
    src/servo_shield.cpp
    src/utils.cpp
    src/settings.cpp

    # Shared headers (not strictly needed but fine to list)
    include/quaid_controller.h
    include/mqtt_controller.h
    include/servo_shield.h
    include/utils.h
    include/settings.h

    ${CMAKE_BINARY_DIR}/paho_fix.cpp
)

target_include_directories(quaid_sim_headless PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${VCPKG_INSTALLED_DIR}/x64-windows/include
    ${MUJOCO_PATH}/include
)

target_link_libraries(quaid_sim_headless
    # MuJoCo
    "${MUJOCO_PATH}/lib/mujoco.lib"

    # MQTT
    "${VCPKG_INSTALLED_DIR}/x64-windows/lib/paho-mqttpp3.lib"
    "${VCPKG_INSTALLED_DIR}/x64-windows/lib/paho-mqtt3as.lib"

    # YAML
    "${VCPKG_INSTALLED_DIR}/x64-windows/lib/yaml-cpp.lib"
)

target_compile_definitions(quaid_sim_headless PRIVATE YAML_CPP_STATIC_DEFINE)
