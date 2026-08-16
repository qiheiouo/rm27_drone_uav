cmake_minimum_required(VERSION 3.16)
project(rm_drone_nav C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# MCU 兼容核心 + host 仿真。所有代码 C99、静态内存、无动态分配。
add_library(nav_core STATIC
    core/planner/waypoint.c
    core/state_estimator/state_estimator.c
    core/mission/mission_fsm.c
    core/guidance/guidance.c
    core/safety/impact_detector.c
    core/safety/safety_monitor.c
    perception/target_tracker/target_tracker.c
    perception/home_detector/home_detector.c
    perception/camera/camera.c
    perception/vision_frontend/vision_frontend.c
    flight/controller_interface/pos_controller.c
    swarm/collision_interface/collision_interface.c
    simulation/scenario.c
    simulation/sim_dynamics.c
    simulation/sim_sensors.c
    simulation/sim_vision.c
)

target_include_directories(nav_core PUBLIC
    core/math
    core/filters
    core/planner
    core/state_estimator
    core/mission
    core/guidance
    core/safety
    perception/target_tracker
    perception/home_detector
    perception/vision_frontend
    perception/camera
    flight/controller_interface
    flight/flight_state
    swarm/agent_state
    swarm/trajectory_message
    swarm/collision_interface
    simulation
)

if(MSVC)
    target_compile_options(nav_core PRIVATE /W4)
else()
    target_compile_options(nav_core PRIVATE -Wall -Wextra)
endif()

if(UNIX)
    target_link_libraries(nav_core PUBLIC m)
endif()

add_executable(mission_sim platform/host/main.c)
target_link_libraries(mission_sim PRIVATE nav_core)

enable_testing()

foreach(t test_math test_impact_detector test_mission_fsm test_camera test_ins_estimator test_vision_frontend)
    add_executable(${t} tests/${t}.c)
    target_link_libraries(${t} PRIVATE nav_core)
    add_test(NAME ${t} COMMAND ${t})
endforeach()

# 完整闭环作为集成测试：必须输出 MISSION_SUCCESS
add_test(NAME mission_closed_loop COMMAND mission_sim)
set_tests_properties(mission_closed_loop PROPERTIES
    PASS_REGULAR_EXPRESSION "MISSION_SUCCESS")

# 蒙特卡洛回归：多种子（偏置/漂移/噪声不同）+ 最坏情况 + 理想对照
foreach(seed 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16)
    add_test(NAME mc_seed_${seed} COMMAND mission_sim --seed ${seed})
    set_tests_properties(mc_seed_${seed} PROPERTIES
        PASS_REGULAR_EXPRESSION "MISSION_SUCCESS")
endforeach()
foreach(seed 11 12 13 14 15 16 17 18 19 20)
    add_test(NAME mc_lost_${seed} COMMAND mission_sim --lost-on-impact --seed ${seed})
    set_tests_properties(mc_lost_${seed} PROPERTIES
        PASS_REGULAR_EXPRESSION "MISSION_SUCCESS")
endforeach()
add_test(NAME mc_truth_mode COMMAND mission_sim --truth-mode --seed 5)
set_tests_properties(mc_truth_mode PROPERTIES
    PASS_REGULAR_EXPRESSION "MISSION_SUCCESS")
