cmake_minimum_required(VERSION 3.16)
project(rm_drone_nav C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(NAV_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

if(MSVC)
    add_compile_options(/W4)
    if(NAV_WARNINGS_AS_ERRORS)
        add_compile_options(/WX)
    endif()
else()
    add_compile_options(-Wall -Wextra)
    if(NAV_WARNINGS_AS_ERRORS)
        add_compile_options(-Werror)
    endif()
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# MCU 兼容核心 + host 仿真。所有代码 C99、静态内存、无动态分配。
add_library(nav_core STATIC
    core/diagnostics/nav_event_log.c
    core/diagnostics/nav_telemetry.c
    core/runtime/nav_runtime.c
    core/planner/waypoint.c
    core/planner/trajectory.c
    core/planner/trajectory_planner.c
    core/state_estimator/state_estimator.c
    core/mission/mission_fsm.c
    core/guidance/guidance.c
    core/safety/impact_detector.c
    core/safety/impact_recovery.c
    core/safety/obstacle_avoidance.c
    core/safety/safety_monitor.c
    perception/target_tracker/target_tracker.c
    perception/home_detector/home_detector.c
    perception/camera/camera.c
    perception/vision_frontend/vision_frontend.c
    flight/controller_interface/pos_controller.c
    flight/fcu_bridge/fcu_bridge.c
    swarm/collision_interface/collision_interface.c
    swarm/collision_interface/swarm_avoidance.c
    swarm/link/swarm_link.c
    swarm/link/swarm_link_guard.c
    swarm/trajectory_message/trajectory_message.c
)

add_library(nav_sim STATIC
    simulation/fault_injection.c
    simulation/scenario.c
    simulation/sim_dynamics.c
    simulation/sim_sensors.c
    simulation/sim_vision.c
    simulation/swarm_network.c
)

target_include_directories(nav_core PUBLIC
    core/diagnostics
    core/runtime
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
    flight/fcu_bridge
    flight/flight_state
    swarm/agent_state
    swarm/link
    swarm/trajectory_message
    swarm/collision_interface
)

target_include_directories(nav_sim PUBLIC simulation)
target_link_libraries(nav_sim PUBLIC nav_core)

if(UNIX)
    target_link_libraries(nav_core PUBLIC m)
endif()

add_executable(mission_sim platform/host/main.c)
target_link_libraries(mission_sim PRIVATE nav_sim)

add_executable(swarm_sim platform/host/swarm_sim.c)
target_link_libraries(swarm_sim PRIVATE nav_sim)

add_executable(nav_replay platform/host/replay.c)
target_link_libraries(nav_replay PRIVATE nav_core)

add_executable(nav_telemetry_dump platform/host/telemetry_dump.c)
target_link_libraries(nav_telemetry_dump PRIVATE nav_core)

add_executable(fcu_bridge_sim platform/host/fcu_bridge_sim.c)
target_link_libraries(fcu_bridge_sim PRIVATE nav_core)

# STM32 移植层骨架：host 编译检查（板级函数仅声明，静态库无需链接）
add_library(nav_stm32_port STATIC platform/stm32/nav_tasks.c)
target_include_directories(nav_stm32_port PUBLIC platform/stm32)
target_link_libraries(nav_stm32_port PUBLIC nav_core)

enable_testing()

foreach(t
    test_math
    test_impact_detector
    test_mission_fsm
    test_camera
    test_ins_estimator
    test_vision_frontend
    test_trajectory
    test_tracking_guidance
    test_recovery_safety
    test_planner_avoidance
    test_swarm
    test_swarm_link
    test_swarm_link_guard
    test_swarm_network
    test_nav_runtime
    test_diagnostics
    test_fault_injection
    test_fcu_bridge
)
    add_executable(${t} tests/${t}.c)
    target_link_libraries(${t} PRIVATE nav_core nav_sim)
    add_test(NAME ${t} COMMAND ${t})
endforeach()

add_test(NAME fcu_bridge_uart_outage COMMAND fcu_bridge_sim)
set_tests_properties(fcu_bridge_uart_outage PROPERTIES
    PASS_REGULAR_EXPRESSION "FCU_BRIDGE_SIM_SUCCESS")

add_executable(test_nav_app_fcu tests/test_nav_app_fcu.c)
target_link_libraries(test_nav_app_fcu PRIVATE nav_stm32_port nav_core)
add_test(NAME test_nav_app_fcu COMMAND test_nav_app_fcu)

foreach(scenario
    nominal
    lossy
    outage
    guarded-outage
    four-agent
)
    add_test(NAME swarm_${scenario} COMMAND swarm_sim --scenario ${scenario})
    set_tests_properties(swarm_${scenario} PROPERTIES
        PASS_REGULAR_EXPRESSION "SWARM_SIM_SUCCESS")
endforeach()

add_executable(test_nav_app_swarm tests/test_nav_app_swarm.c)
target_link_libraries(test_nav_app_swarm PRIVATE nav_stm32_port nav_core)
add_test(NAME test_nav_app_swarm COMMAND test_nav_app_swarm)

foreach(scenario
    flow-dropout
    target-freeze
    home-delay
    nan-target
    imu-stale
    imu-duplicate
    imu-rollback
    watchdog-overrun
)
    add_test(NAME fault_${scenario} COMMAND mission_sim --scenario ${scenario})
    set_tests_properties(fault_${scenario} PROPERTIES
        PASS_REGULAR_EXPRESSION "FAULT_REGRESSION_SUCCESS")
endforeach()

add_executable(test_telemetry tests/test_telemetry.c)
target_link_libraries(test_telemetry PRIVATE nav_core)
add_test(NAME test_telemetry COMMAND test_telemetry)

# 完整闭环作为集成测试：必须输出 MISSION_SUCCESS
add_test(NAME mission_closed_loop COMMAND mission_sim)
set_tests_properties(mission_closed_loop PROPERTIES
    PASS_REGULAR_EXPRESSION "MISSION_SUCCESS")

add_test(NAME replay_determinism
    COMMAND nav_replay --verify
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/data/replay_stationary.csv)
set_tests_properties(replay_determinism PROPERTIES
    PASS_REGULAR_EXPRESSION "REPLAY_SUCCESS")

foreach(scenario
    moving-target
    target-loss
    impact-degraded
    impact-lost
    home-initial-hidden
    home-loss
    local-obstacle
    forced-return
    two-agent-conflict
)
    add_test(NAME scenario_${scenario} COMMAND mission_sim --scenario ${scenario})
    set_tests_properties(scenario_${scenario} PROPERTIES
        PASS_REGULAR_EXPRESSION "MISSION_SUCCESS")
endforeach()

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
# 剧烈撞击（大翻倾恢复）
foreach(seed 21 22 23 24 25)
    add_test(NAME mc_hard_${seed} COMMAND mission_sim --hard-impact --seed ${seed})
    set_tests_properties(mc_hard_${seed} PROPERTIES
        PASS_REGULAR_EXPRESSION "MISSION_SUCCESS")
endforeach()
add_test(NAME mc_truth_mode COMMAND mission_sim --truth-mode --seed 5)
set_tests_properties(mc_truth_mode PROPERTIES
    PASS_REGULAR_EXPRESSION "MISSION_SUCCESS")

set(TELEMETRY_TEST_FILE
    ${CMAKE_CURRENT_BINARY_DIR}/mission_telemetry_test.bin)
add_test(NAME telemetry_record
    COMMAND mission_sim --truth-mode --telemetry ${TELEMETRY_TEST_FILE})
set_tests_properties(telemetry_record PROPERTIES
    PASS_REGULAR_EXPRESSION "MISSION_SUCCESS")
add_test(NAME telemetry_validate
    COMMAND nav_telemetry_dump ${TELEMETRY_TEST_FILE})
set_tests_properties(telemetry_validate PROPERTIES
    DEPENDS telemetry_record
    PASS_REGULAR_EXPRESSION "TELEMETRY_OK")

set(FAULT_TELEMETRY_TEST_FILE
    ${CMAKE_CURRENT_BINARY_DIR}/fault_telemetry_test.bin)
add_test(NAME fault_telemetry_record
    COMMAND mission_sim --scenario imu-stale
            --telemetry ${FAULT_TELEMETRY_TEST_FILE})
set_tests_properties(fault_telemetry_record PROPERTIES
    PASS_REGULAR_EXPRESSION "FAULT_REGRESSION_SUCCESS")
add_test(NAME fault_telemetry_validate
    COMMAND nav_telemetry_dump ${FAULT_TELEMETRY_TEST_FILE})
set_tests_properties(fault_telemetry_validate PROPERTIES
    DEPENDS fault_telemetry_record
    PASS_REGULAR_EXPRESSION "TELEMETRY_OK")
