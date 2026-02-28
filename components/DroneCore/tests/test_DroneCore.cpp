/**
 * @file test_DroneCore.cpp
 * @brief Integration tests for the full drone component stack.
 */
/*
FATP_META:
  meta_version: 1
  component: DroneCore
  file_role: test
  path: components/DroneCore/tests/test_DroneCore.cpp
  namespace: fat_p::testing::dronecore
  layer: Testing
  summary: Integration tests for the full drone component stack.
  api_stability: in_work
  related:
    headers:
      - include/drone/CommandParser.h
      - include/drone/VehicleStateMachine.h
      - include/drone/SubsystemManager.h
      - include/drone/TelemetryLog.h
      - include/drone/DroneEvents.h
  hygiene:
    pragma_once: false
    include_guard: false
    defines_total: 0
    defines_unprefixed: 0
    undefs_total: 0
    includes_windows_h: false
*/

#include <iostream>
#include <string>

#include "CommandParser.h"
#include "DroneEvents.h"
#include "FatPTest.h"
#include "SubsystemManager.h"
#include "Subsystems.h"
#include "TelemetryLog.h"
#include "VehicleStateMachine.h"

namespace fat_p::testing::dronecore
{

struct FullStack
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager      mgr{hub};
    drone::VehicleStateMachine   sm{mgr, hub};
    drone::TelemetryLog<256>     log{hub};
    drone::CommandParser<256>    cmd{mgr, sm, log};

    void enableArmingAndManual()
    {
        using namespace drone::subsystems;
        (void)mgr.enableSubsystem(kIMU);
        (void)mgr.enableSubsystem(kBarometer);
        (void)mgr.enableSubsystem(kBatteryMonitor);
        (void)mgr.enableSubsystem(kESC);
        (void)mgr.enableSubsystem(kMotorMix);
        (void)mgr.enableSubsystem(kRCReceiver);
        (void)mgr.enableSubsystem(kManual);
    }
};

FATP_TEST_CASE(command_unknown_returns_error)
{
    FullStack f;
    auto result = f.cmd.execute("frobnicate");
    FATP_ASSERT_FALSE(result.success, "Unknown command should return failure");
    FATP_ASSERT_CONTAINS(result.message, "Unknown command", "Error should say 'Unknown command'");
    return true;
}

FATP_TEST_CASE(command_empty_line_ok)
{
    FullStack f;
    auto result = f.cmd.execute("");
    FATP_ASSERT_TRUE(result.success, "Empty line should succeed (no-op)");
    return true;
}

FATP_TEST_CASE(command_help_returns_text)
{
    FullStack f;
    auto result = f.cmd.execute("help");
    FATP_ASSERT_TRUE(result.success, "help should succeed");
    FATP_ASSERT_CONTAINS(result.message, "enable",  "Help should list 'enable'");
    FATP_ASSERT_CONTAINS(result.message, "arm",     "Help should list 'arm'");
    FATP_ASSERT_CONTAINS(result.message, "takeoff", "Help should list 'takeoff'");
    return true;
}

FATP_TEST_CASE(command_quit_sets_quit_flag)
{
    FullStack f;
    FATP_ASSERT_TRUE(f.cmd.execute("quit").quit, "quit should set quit flag");
    FATP_ASSERT_TRUE(f.cmd.execute("exit").quit, "exit should set quit flag");
    return true;
}

FATP_TEST_CASE(command_enable_success)
{
    FullStack f;
    auto result = f.cmd.execute("enable GPS");
    FATP_ASSERT_TRUE(result.success, "enable GPS should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled("GPS"), "GPS should be enabled");
    return true;
}

FATP_TEST_CASE(command_enable_missing_arg)
{
    FullStack f;
    auto result = f.cmd.execute("enable");
    FATP_ASSERT_FALSE(result.success, "enable without arg should fail");
    FATP_ASSERT_CONTAINS(result.message, "Usage", "Error should contain usage hint");
    return true;
}

FATP_TEST_CASE(command_enable_dependency_failure)
{
    // FeatureManager Requires = auto-enable, so enabling a subsystem with deps
    // cannot fail due to missing deps alone. The correct conflict scenario is
    // trying to enable a second flight mode while one is already active
    // (MutuallyExclusive constraint).
    FullStack f;
    (void)f.cmd.execute("enable Manual");
    auto result = f.cmd.execute("enable Stabilize"); // Stabilize is mutually exclusive with Manual
    FATP_ASSERT_FALSE(result.success, "enable Stabilize while Manual is active should fail");
    FATP_ASSERT_CONTAINS(result.message, "failed", "Error should say 'failed'");
    return true;
}

FATP_TEST_CASE(command_disable_success)
{
    FullStack f;
    (void)f.cmd.execute("enable GPS");
    auto result = f.cmd.execute("disable GPS");
    FATP_ASSERT_TRUE(result.success, "disable GPS should succeed");
    FATP_ASSERT_FALSE(f.mgr.isEnabled("GPS"), "GPS should be disabled");
    return true;
}

FATP_TEST_CASE(command_disable_missing_arg)
{
    FullStack f;
    auto result = f.cmd.execute("disable");
    FATP_ASSERT_FALSE(result.success, "disable without arg should fail");
    return true;
}

FATP_TEST_CASE(command_status_shows_state)
{
    FullStack f;
    auto result = f.cmd.execute("status");
    FATP_ASSERT_TRUE(result.success, "status should succeed");
    FATP_ASSERT_CONTAINS(result.message, "Preflight", "Status should show current state");
    return true;
}

FATP_TEST_CASE(command_arm_without_subsystems_fails)
{
    FullStack f;
    auto result = f.cmd.execute("arm");
    FATP_ASSERT_FALSE(result.success, "arm without subsystems should fail");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain in Preflight");
    return true;
}

FATP_TEST_CASE(command_arm_success)
{
    FullStack f;
    f.enableArmingAndManual();
    auto result = f.cmd.execute("arm");
    FATP_ASSERT_TRUE(result.success, "arm should succeed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should be Armed");
    return true;
}

FATP_TEST_CASE(command_full_flight_sequence)
{
    FullStack f;
    f.enableArmingAndManual();

    FATP_ASSERT_TRUE(f.cmd.execute("arm").success,              "arm should succeed");
    FATP_ASSERT_TRUE(f.cmd.execute("takeoff").success,          "takeoff should succeed");
    FATP_ASSERT_TRUE(f.cmd.execute("land").success,             "land should succeed");
    FATP_ASSERT_TRUE(f.cmd.execute("landing_complete").success, "landing_complete should succeed");
    FATP_ASSERT_TRUE(f.cmd.execute("disarm").success,           "disarm should succeed");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should end in Preflight");
    return true;
}

FATP_TEST_CASE(command_emergency_and_reset)
{
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");

    FATP_ASSERT_TRUE(f.cmd.execute("emergency engine failure").success, "emergency should succeed");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "Should be Emergency");
    FATP_ASSERT_TRUE(f.cmd.execute("reset").success, "reset should succeed");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should be back in Preflight");
    return true;
}

FATP_TEST_CASE(command_log_after_events)
{
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");

    auto result = f.cmd.execute("log 20");
    FATP_ASSERT_TRUE(result.success, "log should succeed");
    FATP_ASSERT_CONTAINS(result.message, "Armed", "Log should contain Armed transition");
    return true;
}

FATP_TEST_CASE(command_log_invalid_n)
{
    FullStack f;
    FATP_ASSERT_FALSE(f.cmd.execute("log abc").success, "log with non-integer arg should fail");
    return true;
}

FATP_TEST_CASE(command_graph_returns_dot)
{
    FullStack f;
    auto result = f.cmd.execute("graph");
    FATP_ASSERT_TRUE(result.success, "graph should succeed");
    FATP_ASSERT_CONTAINS(result.message, "digraph", "graph should contain 'digraph'");
    return true;
}

FATP_TEST_CASE(command_json_returns_json)
{
    FullStack f;
    (void)f.cmd.execute("enable IMU");
    auto result = f.cmd.execute("json");
    FATP_ASSERT_TRUE(result.success, "json should succeed");
    FATP_ASSERT_CONTAINS(result.message, "IMU", "JSON should contain IMU");
    return true;
}

FATP_TEST_CASE(integration_telemetry_captures_full_flight)
{
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");
    (void)f.cmd.execute("takeoff");
    (void)f.cmd.execute("land");
    (void)f.cmd.execute("landing_complete");
    (void)f.cmd.execute("disarm");

    FATP_ASSERT_FALSE(f.log.empty(), "TelemetryLog should have entries after flight");

    const std::string fmt = f.log.formatTail(50);
    FATP_ASSERT_CONTAINS(fmt, "Armed",   "Log should record Armed");
    FATP_ASSERT_CONTAINS(fmt, "Flying",  "Log should record Flying");
    FATP_ASSERT_CONTAINS(fmt, "Landing", "Log should record Landing");
    return true;
}

FATP_TEST_CASE(integration_safety_alert_in_telemetry)
{
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");
    (void)f.cmd.execute("emergency battery low");

    FATP_ASSERT_CONTAINS(f.log.formatTail(50), "SAFETY",
                         "Telemetry should contain SAFETY category");
    return true;
}

FATP_TEST_CASE(integration_case_insensitive_command)
{
    FullStack f;
    auto result = f.cmd.execute("HELP");
    FATP_ASSERT_TRUE(result.success, "HELP uppercase should work");
    FATP_ASSERT_CONTAINS(result.message, "enable", "Help text should list enable");
    return true;
}

} // namespace fat_p::testing::dronecore

namespace fat_p::testing
{

bool test_DroneCore()
{
    FATP_PRINT_HEADER(DRONE CORE INTEGRATION)

    TestRunner runner;

    FATP_RUN_TEST_NS(runner, dronecore, command_unknown_returns_error);
    FATP_RUN_TEST_NS(runner, dronecore, command_empty_line_ok);
    FATP_RUN_TEST_NS(runner, dronecore, command_help_returns_text);
    FATP_RUN_TEST_NS(runner, dronecore, command_quit_sets_quit_flag);
    FATP_RUN_TEST_NS(runner, dronecore, command_enable_success);
    FATP_RUN_TEST_NS(runner, dronecore, command_enable_missing_arg);
    FATP_RUN_TEST_NS(runner, dronecore, command_enable_dependency_failure);
    FATP_RUN_TEST_NS(runner, dronecore, command_disable_success);
    FATP_RUN_TEST_NS(runner, dronecore, command_disable_missing_arg);
    FATP_RUN_TEST_NS(runner, dronecore, command_status_shows_state);
    FATP_RUN_TEST_NS(runner, dronecore, command_arm_without_subsystems_fails);
    FATP_RUN_TEST_NS(runner, dronecore, command_arm_success);
    FATP_RUN_TEST_NS(runner, dronecore, command_full_flight_sequence);
    FATP_RUN_TEST_NS(runner, dronecore, command_emergency_and_reset);
    FATP_RUN_TEST_NS(runner, dronecore, command_log_after_events);
    FATP_RUN_TEST_NS(runner, dronecore, command_log_invalid_n);
    FATP_RUN_TEST_NS(runner, dronecore, command_graph_returns_dot);
    FATP_RUN_TEST_NS(runner, dronecore, command_json_returns_json);
    FATP_RUN_TEST_NS(runner, dronecore, integration_telemetry_captures_full_flight);
    FATP_RUN_TEST_NS(runner, dronecore, integration_safety_alert_in_telemetry);
    FATP_RUN_TEST_NS(runner, dronecore, integration_case_insensitive_command);

    return 0 == runner.print_summary();
}

} // namespace fat_p::testing

#ifdef ENABLE_TEST_APPLICATION
int main()
{
    return fat_p::testing::test_DroneCore() ? 0 : 1;
}
#endif
