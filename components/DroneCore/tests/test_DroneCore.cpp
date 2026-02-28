/**
 * @file test_DroneCore.cpp
 * @brief Integration tests for the full drone component stack.
 *
 * Tests cover: all CLI commands, happy path sequences, adversarial inputs
 * (wrong state, bad args, hostile strings), disarm_after_landing command,
 * and stress sequences through the CommandParser.
 */
/*
FATP_META:
  meta_version: 1
  component: DroneCore
  file_role: test
  path: components/DroneCore/tests/test_DroneCore.cpp
  namespace: fat_p::testing::dronecore
  layer: Testing
  summary: Integration tests for the full drone stack - commands, adversarial, stress.
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

    void goFlying()
    {
        enableArmingAndManual();
        (void)cmd.execute("arm");
        (void)cmd.execute("takeoff");
    }

    void goLanding()
    {
        goFlying();
        (void)cmd.execute("land");
    }
};

// ============================================================================
// Basic commands
// ============================================================================

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
    FATP_ASSERT_TRUE(f.cmd.execute("").success, "Empty line should succeed (no-op)");
    return true;
}

FATP_TEST_CASE(command_help_returns_text)
{
    FullStack f;
    auto result = f.cmd.execute("help");
    FATP_ASSERT_TRUE(result.success, "help should succeed");
    FATP_ASSERT_CONTAINS(result.message, "enable",              "Help should list 'enable'");
    FATP_ASSERT_CONTAINS(result.message, "arm",                 "Help should list 'arm'");
    FATP_ASSERT_CONTAINS(result.message, "takeoff",             "Help should list 'takeoff'");
    FATP_ASSERT_CONTAINS(result.message, "disarm_after_landing","Help should list 'disarm_after_landing'");
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
    FullStack f;
    (void)f.cmd.execute("enable Manual");
    auto result = f.cmd.execute("enable Stabilize"); // MutuallyExclusive conflict
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
    FATP_ASSERT_FALSE(f.cmd.execute("disable").success, "disable without arg should fail");
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
    FATP_ASSERT_TRUE(f.cmd.execute("arm").success, "arm should succeed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should be Armed");
    return true;
}

FATP_TEST_CASE(command_full_flight_sequence)
{
    FullStack f;
    f.enableArmingAndManual();
    FATP_ASSERT_TRUE(f.cmd.execute("arm").success,              "arm");
    FATP_ASSERT_TRUE(f.cmd.execute("takeoff").success,          "takeoff");
    FATP_ASSERT_TRUE(f.cmd.execute("land").success,             "land");
    FATP_ASSERT_TRUE(f.cmd.execute("landing_complete").success, "landing_complete");
    FATP_ASSERT_TRUE(f.cmd.execute("disarm").success,           "disarm");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should end in Preflight");
    return true;
}

FATP_TEST_CASE(command_disarm_after_landing)
{
    // Landing -> Preflight directly via the disarm_after_landing command.
    FullStack f;
    f.goLanding();
    FATP_ASSERT_TRUE(f.sm.isLanding(), "Pre-condition: must be Landing");
    auto result = f.cmd.execute("disarm_after_landing");
    FATP_ASSERT_TRUE(result.success, "disarm_after_landing should succeed from Landing");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should be Preflight after disarm_after_landing");
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

// ============================================================================
// Integration
// ============================================================================

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

// ============================================================================
// Adversarial — wrong-state commands and hostile inputs
// ============================================================================

FATP_TEST_CASE(adversarial_takeoff_from_preflight_fails)
{
    FullStack f;
    auto res = f.cmd.execute("takeoff");
    FATP_ASSERT_FALSE(res.success, "takeoff should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain Preflight");
    return true;
}

FATP_TEST_CASE(adversarial_land_from_preflight_fails)
{
    FullStack f;
    FATP_ASSERT_FALSE(f.cmd.execute("land").success, "land should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain Preflight");
    return true;
}

FATP_TEST_CASE(adversarial_land_from_armed_fails)
{
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");
    FATP_ASSERT_FALSE(f.cmd.execute("land").success, "land should fail from Armed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

FATP_TEST_CASE(adversarial_landing_complete_from_armed_fails)
{
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");
    FATP_ASSERT_FALSE(f.cmd.execute("landing_complete").success,
                      "landing_complete should fail from Armed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

FATP_TEST_CASE(adversarial_disarm_after_landing_from_armed_fails)
{
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");
    FATP_ASSERT_FALSE(f.cmd.execute("disarm_after_landing").success,
                      "disarm_after_landing should fail from Armed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

FATP_TEST_CASE(adversarial_disarm_after_landing_from_preflight_fails)
{
    FullStack f;
    FATP_ASSERT_FALSE(f.cmd.execute("disarm_after_landing").success,
                      "disarm_after_landing should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain Preflight");
    return true;
}

FATP_TEST_CASE(adversarial_emergency_from_preflight_fails)
{
    FullStack f;
    FATP_ASSERT_FALSE(f.cmd.execute("emergency fire").success,
                      "emergency should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain Preflight");
    return true;
}

FATP_TEST_CASE(adversarial_reset_from_preflight_fails)
{
    FullStack f;
    FATP_ASSERT_FALSE(f.cmd.execute("reset").success, "reset should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain Preflight");
    return true;
}

FATP_TEST_CASE(adversarial_disarm_from_flying_fails)
{
    FullStack f;
    f.goFlying();
    FATP_ASSERT_FALSE(f.cmd.execute("disarm").success, "disarm should fail from Flying");
    FATP_ASSERT_TRUE(f.sm.isFlying(), "Should remain Flying");
    return true;
}

FATP_TEST_CASE(adversarial_arm_when_already_armed_fails)
{
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");
    FATP_ASSERT_FALSE(f.cmd.execute("arm").success, "arm should fail when already Armed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

FATP_TEST_CASE(adversarial_enable_unknown_subsystem)
{
    FullStack f;
    auto res = f.cmd.execute("enable XYZZY");
    FATP_ASSERT_FALSE(res.success, "Enabling unknown subsystem should fail");
    return true;
}

FATP_TEST_CASE(adversarial_disable_unknown_subsystem)
{
    FullStack f;
    FATP_ASSERT_FALSE(f.cmd.execute("disable XYZZY").success,
                      "Disabling unknown subsystem should fail");
    return true;
}

FATP_TEST_CASE(adversarial_log_zero)
{
    // log 0 should succeed but produce empty-or-minimal output without crashing.
    FullStack f;
    auto res = f.cmd.execute("log 0");
    FATP_ASSERT_TRUE(res.success, "log 0 should not crash");
    return true;
}

FATP_TEST_CASE(adversarial_log_huge_n)
{
    // log with a very large N should succeed (clamped to log size).
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");
    auto res = f.cmd.execute("log 999999");
    FATP_ASSERT_TRUE(res.success, "log with huge n should succeed (clamped)");
    return true;
}

FATP_TEST_CASE(adversarial_command_with_leading_whitespace)
{
    // Leading spaces before command token — parser trims first token.
    // Depending on implementation this may be "unknown" or succeed after trim;
    // either is valid. Critical: must not crash.
    FullStack f;
    auto res = f.cmd.execute("   help");
    // Result may be success or unknown-command — what matters is no crash and
    // the quit flag is false.
    FATP_ASSERT_FALSE(res.quit, "Leading-space command must not set quit flag");
    return true;
}

FATP_TEST_CASE(adversarial_very_long_subsystem_name)
{
    // A pathologically long name should fail cleanly, not crash or corrupt state.
    FullStack f;
    std::string longName(4096, 'A');
    auto res = f.cmd.execute("enable " + longName);
    FATP_ASSERT_FALSE(res.success, "Enabling a 4096-char unknown name should fail");
    return true;
}

FATP_TEST_CASE(adversarial_null_byte_in_command)
{
    // A command string containing an embedded null — should fail cleanly.
    FullStack f;
    std::string evil("enabl\0e GPS", 11);
    auto res = f.cmd.execute(evil);
    // Anything except a crash/UB is acceptable.
    FATP_ASSERT_FALSE(res.quit, "Null-byte command must not set quit flag");
    return true;
}

FATP_TEST_CASE(adversarial_takeoff_no_flight_mode_error_message)
{
    // Takeoff should fail with a message explaining what's missing.
    FullStack f;
    f.enableArmingAndManual();
    (void)f.cmd.execute("arm");
    // Disable Manual so no flight mode is active
    (void)f.mgr.disableSubsystem("Manual");
    auto res = f.cmd.execute("takeoff");
    FATP_ASSERT_FALSE(res.success, "Takeoff without flight mode should fail");
    FATP_ASSERT_FALSE(res.message.empty(), "Failure message should not be empty");
    return true;
}

// ============================================================================
// Stress
// ============================================================================

FATP_TEST_CASE(stress_repeated_full_flight_via_commands)
{
    for (int i = 0; i < 10; ++i)
    {
        FullStack f;
        f.enableArmingAndManual();
        FATP_ASSERT_TRUE(f.cmd.execute("arm").success,              "arm");
        FATP_ASSERT_TRUE(f.cmd.execute("takeoff").success,          "takeoff");
        FATP_ASSERT_TRUE(f.cmd.execute("land").success,             "land");
        FATP_ASSERT_TRUE(f.cmd.execute("landing_complete").success, "landing_complete");
        FATP_ASSERT_TRUE(f.cmd.execute("disarm").success,           "disarm");
        FATP_ASSERT_TRUE(f.sm.isPreflight(),                        "Preflight at end");
    }
    return true;
}

FATP_TEST_CASE(stress_disarm_after_landing_via_commands)
{
    for (int i = 0; i < 10; ++i)
    {
        FullStack f;
        f.goLanding();
        FATP_ASSERT_TRUE(f.cmd.execute("disarm_after_landing").success, "disarm_after_landing");
        FATP_ASSERT_TRUE(f.sm.isPreflight(), "Preflight at end");
    }
    return true;
}

FATP_TEST_CASE(stress_emergency_reset_via_commands)
{
    for (int i = 0; i < 10; ++i)
    {
        FullStack f;
        f.enableArmingAndManual();
        (void)f.cmd.execute("arm");
        (void)f.cmd.execute("takeoff");
        FATP_ASSERT_TRUE(f.cmd.execute("emergency stress test").success, "emergency");
        FATP_ASSERT_TRUE(f.cmd.execute("reset").success,                 "reset");
        FATP_ASSERT_TRUE(f.sm.isPreflight(), "Preflight after reset");
    }
    return true;
}

FATP_TEST_CASE(stress_rejected_commands_do_not_corrupt_state)
{
    // Fire a barrage of wrong-state commands; state must remain Preflight throughout.
    FullStack f;
    static constexpr const char* kWrongCmds[] = {
        "disarm", "takeoff", "land", "landing_complete",
        "disarm_after_landing", "emergency bad", "reset"
    };
    for (int i = 0; i < 20; ++i)
    {
        for (const char* c : kWrongCmds)
        {
            (void)f.cmd.execute(c);
        }
        FATP_ASSERT_TRUE(f.sm.isPreflight(), "State must remain Preflight throughout barrage");
    }
    return true;
}

FATP_TEST_CASE(stress_telemetry_log_fills_and_caps)
{
    // Drive enough events to exercise the rolling eviction in TelemetryLog<256>.
    drone::events::DroneEventHub hub;
    drone::SubsystemManager      mgr{hub};
    drone::VehicleStateMachine   sm{mgr, hub};
    drone::TelemetryLog<16>      log{hub};  // tiny capacity to force eviction
    drone::CommandParser<16>     cmd{mgr, sm, log};

    using namespace drone::subsystems;
    (void)mgr.enableSubsystem(kIMU);
    (void)mgr.enableSubsystem(kBarometer);
    (void)mgr.enableSubsystem(kMotorMix);
    (void)mgr.enableSubsystem(kRCReceiver);
    (void)mgr.enableSubsystem(kManual);

    for (int i = 0; i < 30; ++i)
    {
        (void)cmd.execute("arm");
        (void)cmd.execute("takeoff");
        (void)cmd.execute("land");
        (void)cmd.execute("landing_complete");
        (void)cmd.execute("disarm");
    }

    FATP_ASSERT_LE(log.size(), std::size_t(16), "Log must not exceed MaxEntries=16");
    FATP_ASSERT_FALSE(log.empty(), "Log must not be empty after activity");
    return true;
}

} // namespace fat_p::testing::dronecore

namespace fat_p::testing
{

bool test_DroneCore()
{
    FATP_PRINT_HEADER(DRONE CORE INTEGRATION)

    TestRunner runner;

    // Basic commands
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
    FATP_RUN_TEST_NS(runner, dronecore, command_disarm_after_landing);
    FATP_RUN_TEST_NS(runner, dronecore, command_emergency_and_reset);
    FATP_RUN_TEST_NS(runner, dronecore, command_log_after_events);
    FATP_RUN_TEST_NS(runner, dronecore, command_log_invalid_n);
    FATP_RUN_TEST_NS(runner, dronecore, command_graph_returns_dot);
    FATP_RUN_TEST_NS(runner, dronecore, command_json_returns_json);

    // Integration
    FATP_RUN_TEST_NS(runner, dronecore, integration_telemetry_captures_full_flight);
    FATP_RUN_TEST_NS(runner, dronecore, integration_safety_alert_in_telemetry);
    FATP_RUN_TEST_NS(runner, dronecore, integration_case_insensitive_command);

    // Adversarial
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_takeoff_from_preflight_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_land_from_preflight_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_land_from_armed_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_landing_complete_from_armed_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_disarm_after_landing_from_armed_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_disarm_after_landing_from_preflight_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_emergency_from_preflight_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_reset_from_preflight_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_disarm_from_flying_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_arm_when_already_armed_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_enable_unknown_subsystem);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_disable_unknown_subsystem);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_log_zero);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_log_huge_n);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_command_with_leading_whitespace);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_very_long_subsystem_name);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_null_byte_in_command);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_takeoff_no_flight_mode_error_message);

    // Stress
    FATP_RUN_TEST_NS(runner, dronecore, stress_repeated_full_flight_via_commands);
    FATP_RUN_TEST_NS(runner, dronecore, stress_disarm_after_landing_via_commands);
    FATP_RUN_TEST_NS(runner, dronecore, stress_emergency_reset_via_commands);
    FATP_RUN_TEST_NS(runner, dronecore, stress_rejected_commands_do_not_corrupt_state);
    FATP_RUN_TEST_NS(runner, dronecore, stress_telemetry_log_fills_and_caps);

    return 0 == runner.print_summary();
}

} // namespace fat_p::testing

#ifdef ENABLE_TEST_APPLICATION
int main()
{
    return fat_p::testing::test_DroneCore() ? 0 : 1;
}
#endif
