/**
 * @file test_DroneCore.cpp
 * @brief Integration tests for the full drone component stack.
 *
 * Graph-native design invariants verified end-to-end:
 *   - arm brings up power chain via ArmedProfile Entails (user need not enable BatteryMonitor/ESC/MotorMix)
 *   - disarm / disarm_after_landing / emergency reset all land in canonical Preflight via graph
 *   - airborne emergency uses EmergencyLandProfile Entails to keep motors live
 *   - resetEmergencyStop auto-cleans power chain through FM Entails ref-count
 *   - reserved features (EmergencyStop, ArmedProfile, EmergencyLandProfile) blocked from raw toggle
 *   - leading-whitespace trimming, case normalisation, adversarial inputs
 */
/*
FATP_META:
  meta_version: 1
  component: DroneCore
  file_role: test
  path: components/DroneCore/tests/test_DroneCore.cpp
  namespace: fat_p::testing::dronecore
  layer: Testing
  api_stability: in_work
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

    // Minimal arming prerequisites — power chain is NOT set up by user;
    // it is brought up by ArmedProfile Entails when arm() fires.
    void enableArmingPrereqs()
    {
        using namespace drone::subsystems;
        (void)mgr.enableSubsystem(kIMU);
        (void)mgr.enableSubsystem(kBarometer);
        (void)mgr.enableSubsystem(kRCReceiver);
        (void)mgr.enableSubsystem(kManual);
    }

    void goFlying()
    {
        enableArmingPrereqs();
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
// Graph-native: arm brings up power chain via FM Entails
// ============================================================================

FATP_TEST_CASE(arm_brings_up_power_chain_via_graph)
{
    // User only enables IMU, Barometer, RCReceiver, Manual.
    // Power chain must be absent before arming and present after.
    FullStack f;
    using namespace drone::subsystems;
    f.enableArmingPrereqs();

    FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix),       "MotorMix off before arm");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),            "ESC off before arm");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kBatteryMonitor), "BatteryMonitor off before arm");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileArmed),   "ArmedProfile off before arm");

    FATP_ASSERT_TRUE(f.cmd.execute("arm").success, "arm should succeed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "should be Armed");

    FATP_ASSERT_TRUE(f.mgr.isEnabled(kProfileArmed),   "ArmedProfile on after arm");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix),       "MotorMix auto-up via Entails");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kESC),            "ESC auto-up via Requires");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBatteryMonitor), "BatteryMonitor auto-up via Requires");
    return true;
}

FATP_TEST_CASE(disarm_tears_down_power_chain_via_graph)
{
    FullStack f;
    using namespace drone::subsystems;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix), "MotorMix on after arm");

    FATP_ASSERT_TRUE(f.cmd.execute("disarm").success, "disarm should succeed");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "should be Preflight");

    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileArmed), "ArmedProfile cleared");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix),     "MotorMix auto-torn-down via Entails");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),          "ESC auto-torn-down");
    FATP_ASSERT_TRUE(f.mgr.activeFlightMode().empty(), "No active flight mode");
    return true;
}

FATP_TEST_CASE(disarm_after_landing_tears_down_power_chain_via_graph)
{
    FullStack f;
    using namespace drone::subsystems;
    f.goLanding();
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix), "MotorMix on during landing");

    FATP_ASSERT_TRUE(f.cmd.execute("disarm_after_landing").success, "f.cmd.execute(\"disarm_after_landing\").success");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix),    "MotorMix auto-torn-down");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),         "ESC auto-torn-down");
    FATP_ASSERT_TRUE(f.mgr.activeFlightMode().empty(), "f.mgr.activeFlightMode().empty()");
    return true;
}

FATP_TEST_CASE(airborne_emergency_uses_emergency_land_profile)
{
    FullStack f;
    using namespace drone::subsystems;
    f.goFlying();
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kProfileArmed), "ArmedProfile on while flying");

    auto res = f.cmd.execute("emergency sensor loss");
    FATP_ASSERT_TRUE(res.success, "emergency should succeed");
    FATP_ASSERT_CONTAINS(res.message, "LAND", "Airborne emergency message must say LAND");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "f.sm.isEmergency()");

    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileArmed),        "ArmedProfile cleared by forceExclusive");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kProfileEmergencyLand), "EmergencyLandProfile owns power chain");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix),             "MotorMix live via EmergencyLandProfile");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kEmergencyStop),        "EmergencyStop latched");
    return true;
}

FATP_TEST_CASE(airborne_emergency_reset_auto_cleans_power_chain_no_explicit_disable)
{
    // After airborne emergency, reset must clean up MotorMix/ESC through the
    // FM graph (EmergencyLandProfile Entails ref-count) — no explicit disable loop.
    FullStack f;
    using namespace drone::subsystems;
    f.goFlying();
    (void)f.cmd.execute("emergency autopilot fail");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix), "MotorMix live during emergency");

    FATP_ASSERT_TRUE(f.cmd.execute("reset").success, "f.cmd.execute(\"reset\").success");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kEmergencyStop),        "ES cleared");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileEmergencyLand), "ELand profile cleared");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix),             "MotorMix auto-torn-down");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),                  "ESC auto-torn-down");
    return true;
}

FATP_TEST_CASE(ground_emergency_message_says_stop)
{
    FullStack f;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    auto res = f.cmd.execute("emergency motor stall");
    FATP_ASSERT_TRUE(res.success, "res.success");
    FATP_ASSERT_CONTAINS(res.message, "STOP", "Ground emergency must say STOP");
    return true;
}

FATP_TEST_CASE(ground_emergency_reset_restores_preflight)
{
    FullStack f;
    using namespace drone::subsystems;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    (void)f.cmd.execute("emergency");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "f.sm.isEmergency()");

    (void)f.cmd.execute("reset");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix), "f.mgr.isEnabled(kMotorMix)");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC), "f.mgr.isEnabled(kESC)");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kEmergencyStop), "f.mgr.isEnabled(kEmergencyStop)");
    return true;
}

// ============================================================================
// Reserved feature guards
// ============================================================================

FATP_TEST_CASE(enable_emergency_stop_raw_blocked)
{
    FullStack f;
    auto res = f.cmd.execute("enable EmergencyStop");
    FATP_ASSERT_FALSE(res.success, "enable EmergencyStop must be rejected");
    FATP_ASSERT_CONTAINS(res.message, "reserved", "res.message, \"reserved\" contains ");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(drone::subsystems::kEmergencyStop), "f.mgr.isEnabled(drone::subsystems::kEmergencyStop)");
    return true;
}

FATP_TEST_CASE(disable_emergency_stop_raw_blocked)
{
    FullStack f;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    (void)f.cmd.execute("emergency test");
    auto res = f.cmd.execute("disable EmergencyStop");
    FATP_ASSERT_FALSE(res.success, "disable EmergencyStop must be rejected");
    FATP_ASSERT_CONTAINS(res.message, "reserved", "res.message, \"reserved\" contains ");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "Must remain in Emergency");
    return true;
}

FATP_TEST_CASE(enable_armed_profile_raw_blocked)
{
    FullStack f;
    auto res = f.cmd.execute("enable ArmedProfile");
    FATP_ASSERT_FALSE(res.success, "enable ArmedProfile must be rejected");
    FATP_ASSERT_CONTAINS(res.message, "reserved", "res.message, \"reserved\" contains ");
    return true;
}

FATP_TEST_CASE(enable_emergency_land_profile_raw_blocked)
{
    FullStack f;
    auto res = f.cmd.execute("enable EmergencyLandProfile");
    FATP_ASSERT_FALSE(res.success, "enable EmergencyLandProfile must be rejected");
    FATP_ASSERT_CONTAINS(res.message, "reserved", "res.message, \"reserved\" contains ");
    return true;
}

// ============================================================================
// Basic commands
// ============================================================================

FATP_TEST_CASE(command_unknown_returns_error)
{
    FullStack f;
    auto result = f.cmd.execute("frobnicate");
    FATP_ASSERT_FALSE(result.success, "result.success");
    FATP_ASSERT_CONTAINS(result.message, "Unknown command", "result.message, \"Unknown command\" contains ");
    return true;
}

FATP_TEST_CASE(command_empty_line_ok)
{
    FullStack f;
    FATP_ASSERT_TRUE(f.cmd.execute("").success, "f.cmd.execute(\"\").success");
    return true;
}

FATP_TEST_CASE(command_help_returns_text)
{
    FullStack f;
    auto result = f.cmd.execute("help");
    FATP_ASSERT_TRUE(result.success, "result.success");
    FATP_ASSERT_CONTAINS(result.message, "enable", "result.message, \"enable\" contains ");
    FATP_ASSERT_CONTAINS(result.message, "arm", "result.message, \"arm\" contains ");
    FATP_ASSERT_CONTAINS(result.message, "disarm_after_landing", "result.message, \"disarm_after_landing\" contains ");
    return true;
}

FATP_TEST_CASE(command_quit_sets_quit_flag)
{
    FullStack f;
    FATP_ASSERT_TRUE(f.cmd.execute("quit").quit, "f.cmd.execute(\"quit\").quit");
    FATP_ASSERT_TRUE(f.cmd.execute("exit").quit, "f.cmd.execute(\"exit\").quit");
    return true;
}

FATP_TEST_CASE(command_enable_success)
{
    FullStack f;
    FATP_ASSERT_TRUE(f.cmd.execute("enable GPS").success, "f.cmd.execute(\"enable GPS\").success");
    FATP_ASSERT_TRUE(f.mgr.isEnabled("GPS"), "f.mgr.isEnabled(\"GPS\")");
    return true;
}

FATP_TEST_CASE(command_enable_missing_arg)
{
    FullStack f;
    auto res = f.cmd.execute("enable");
    FATP_ASSERT_FALSE(res.success, "res.success");
    FATP_ASSERT_CONTAINS(res.message, "Usage", "res.message, \"Usage\" contains ");
    return true;
}

FATP_TEST_CASE(command_disable_success)
{
    FullStack f;
    (void)f.cmd.execute("enable GPS");
    auto res = f.cmd.execute("disable GPS");
    FATP_ASSERT_TRUE(res.success, "res.success");
    FATP_ASSERT_FALSE(f.mgr.isEnabled("GPS"), "f.mgr.isEnabled(\"GPS\")");
    return true;
}

FATP_TEST_CASE(command_status_shows_state)
{
    FullStack f;
    auto result = f.cmd.execute("status");
    FATP_ASSERT_TRUE(result.success, "result.success");
    FATP_ASSERT_CONTAINS(result.message, "Preflight", "result.message, \"Preflight\" contains ");
    return true;
}

FATP_TEST_CASE(command_arm_without_prereqs_fails)
{
    FullStack f;
    FATP_ASSERT_FALSE(f.cmd.execute("arm").success, "f.cmd.execute(\"arm\").success");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    return true;
}

FATP_TEST_CASE(command_arm_success)
{
    FullStack f;
    f.enableArmingPrereqs();
    FATP_ASSERT_TRUE(f.cmd.execute("arm").success, "f.cmd.execute(\"arm\").success");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "f.sm.isArmed()");
    return true;
}

FATP_TEST_CASE(command_full_flight_sequence)
{
    FullStack f;
    f.enableArmingPrereqs();
    FATP_ASSERT_TRUE(f.cmd.execute("arm").success,              "arm");
    FATP_ASSERT_TRUE(f.cmd.execute("takeoff").success,          "takeoff");
    FATP_ASSERT_TRUE(f.cmd.execute("land").success,             "land");
    FATP_ASSERT_TRUE(f.cmd.execute("landing_complete").success, "landing_complete");
    FATP_ASSERT_TRUE(f.cmd.execute("disarm").success,           "disarm");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    return true;
}

FATP_TEST_CASE(command_disarm_after_landing)
{
    FullStack f;
    f.goLanding();
    FATP_ASSERT_TRUE(f.sm.isLanding(), "Pre-condition");
    FATP_ASSERT_TRUE(f.cmd.execute("disarm_after_landing").success, "f.cmd.execute(\"disarm_after_landing\").success");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    return true;
}

FATP_TEST_CASE(command_emergency_and_reset)
{
    FullStack f;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    FATP_ASSERT_TRUE(f.cmd.execute("emergency engine failure").success, "f.cmd.execute(\"emergency engine failure\").success");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "f.sm.isEmergency()");
    FATP_ASSERT_TRUE(f.cmd.execute("reset").success, "f.cmd.execute(\"reset\").success");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    return true;
}

FATP_TEST_CASE(command_log_after_events)
{
    FullStack f;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    auto result = f.cmd.execute("log 20");
    FATP_ASSERT_TRUE(result.success, "result.success");
    FATP_ASSERT_CONTAINS(result.message, "Armed", "result.message, \"Armed\" contains ");
    return true;
}

FATP_TEST_CASE(command_graph_returns_dot)
{
    FullStack f;
    auto result = f.cmd.execute("graph");
    FATP_ASSERT_TRUE(result.success, "result.success");
    FATP_ASSERT_CONTAINS(result.message, "digraph", "result.message, \"digraph\" contains ");
    return true;
}

FATP_TEST_CASE(command_json_returns_json)
{
    FullStack f;
    (void)f.cmd.execute("enable IMU");
    auto result = f.cmd.execute("json");
    FATP_ASSERT_TRUE(result.success, "result.success");
    FATP_ASSERT_CONTAINS(result.message, "IMU", "result.message, \"IMU\" contains ");
    return true;
}

FATP_TEST_CASE(integration_case_insensitive_command)
{
    FullStack f;
    auto result = f.cmd.execute("HELP");
    FATP_ASSERT_TRUE(result.success, "result.success");
    FATP_ASSERT_CONTAINS(result.message, "enable", "result.message, \"enable\" contains ");
    return true;
}

FATP_TEST_CASE(integration_telemetry_captures_full_flight)
{
    FullStack f;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    (void)f.cmd.execute("takeoff");
    (void)f.cmd.execute("land");
    (void)f.cmd.execute("landing_complete");
    (void)f.cmd.execute("disarm");
    FATP_ASSERT_FALSE(f.log.empty(), "f.log.empty()");
    const std::string fmt = f.log.formatTail(50);
    FATP_ASSERT_CONTAINS(fmt, "Armed", "fmt, \"Armed\" contains ");
    FATP_ASSERT_CONTAINS(fmt, "Flying", "fmt, \"Flying\" contains ");
    FATP_ASSERT_CONTAINS(fmt, "Landing", "fmt, \"Landing\" contains ");
    return true;
}

// ============================================================================
// Adversarial
// ============================================================================

FATP_TEST_CASE(adversarial_takeoff_from_preflight_fails)
{
    FullStack f;
    FATP_ASSERT_FALSE(f.cmd.execute("takeoff").success, "f.cmd.execute(\"takeoff\").success");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    return true;
}

FATP_TEST_CASE(adversarial_disarm_from_flying_fails)
{
    FullStack f;
    f.goFlying();
    FATP_ASSERT_FALSE(f.cmd.execute("disarm").success, "f.cmd.execute(\"disarm\").success");
    FATP_ASSERT_TRUE(f.sm.isFlying(), "f.sm.isFlying()");
    return true;
}

FATP_TEST_CASE(adversarial_arm_when_already_armed_fails)
{
    FullStack f;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    FATP_ASSERT_FALSE(f.cmd.execute("arm").success, "f.cmd.execute(\"arm\").success");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "f.sm.isArmed()");
    return true;
}

FATP_TEST_CASE(adversarial_command_with_leading_whitespace)
{
    // execute() trims leading whitespace — "   help" must succeed.
    FullStack f;
    auto res = f.cmd.execute("   help");
    FATP_ASSERT_TRUE(res.success, "Leading-space 'help' must succeed after trim");
    FATP_ASSERT_CONTAINS(res.message, "enable", "Help text must be returned");
    FATP_ASSERT_FALSE(res.quit, "res.quit");
    return true;
}

FATP_TEST_CASE(adversarial_enable_unknown_subsystem)
{
    FullStack f;
    FATP_ASSERT_FALSE(f.cmd.execute("enable XYZZY").success, "f.cmd.execute(\"enable XYZZY\").success");
    return true;
}

FATP_TEST_CASE(adversarial_very_long_subsystem_name)
{
    FullStack f;
    std::string longName(4096, 'A');
    FATP_ASSERT_FALSE(f.cmd.execute("enable " + longName).success, "f.cmd.execute(\"enable \" + longName).success");
    return true;
}

FATP_TEST_CASE(adversarial_null_byte_in_command)
{
    FullStack f;
    std::string evil("enabl\0e GPS", 11);
    auto res = f.cmd.execute(evil);
    FATP_ASSERT_FALSE(res.quit, "res.quit");
    return true;
}

FATP_TEST_CASE(adversarial_takeoff_no_flight_mode_error_message)
{
    FullStack f;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    (void)f.mgr.disableSubsystem("Manual");
    auto res = f.cmd.execute("takeoff");
    FATP_ASSERT_FALSE(res.success, "res.success");
    FATP_ASSERT_FALSE(res.message.empty(), "res.message.empty()");
    return true;
}

FATP_TEST_CASE(adversarial_arm_blocked_while_emergency_stop_active)
{
    // Confirm arm is rejected when EmergencyStop is active.
    FullStack f;
    f.enableArmingPrereqs();
    (void)f.cmd.execute("arm");
    (void)f.cmd.execute("emergency test latch");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "f.sm.isEmergency()");
    // Reset goes back to Preflight.
    (void)f.cmd.execute("reset");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    // ES is cleared after reset; re-enable prereqs and confirm we can arm again.
    (void)f.mgr.enableSubsystem(drone::subsystems::kManual);
    FATP_ASSERT_TRUE(f.cmd.execute("arm").success, "arm should succeed after clean reset");
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
        f.enableArmingPrereqs();
        FATP_ASSERT_TRUE(f.cmd.execute("arm").success,              "arm");
        FATP_ASSERT_TRUE(f.cmd.execute("takeoff").success,          "takeoff");
        FATP_ASSERT_TRUE(f.cmd.execute("land").success,             "land");
        FATP_ASSERT_TRUE(f.cmd.execute("landing_complete").success, "landing_complete");
        FATP_ASSERT_TRUE(f.cmd.execute("disarm").success,           "disarm");
        FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
        FATP_ASSERT_FALSE(f.mgr.isEnabled(drone::subsystems::kMotorMix), "MotorMix off");
        FATP_ASSERT_FALSE(f.mgr.isEnabled(drone::subsystems::kESC),      "ESC off");
    }
    return true;
}

FATP_TEST_CASE(stress_emergency_reset_via_commands)
{
    for (int i = 0; i < 10; ++i)
    {
        FullStack f;
        f.goFlying();
        FATP_ASSERT_TRUE(f.cmd.execute("emergency stress test").success, "f.cmd.execute(\"emergency stress test\").success");
        FATP_ASSERT_TRUE(f.mgr.isEnabled(drone::subsystems::kMotorMix), "motors live");
        FATP_ASSERT_TRUE(f.cmd.execute("reset").success, "f.cmd.execute(\"reset\").success");
        FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
        FATP_ASSERT_FALSE(f.mgr.isEnabled(drone::subsystems::kMotorMix), "motors off after reset");
        FATP_ASSERT_FALSE(f.mgr.isEnabled(drone::subsystems::kESC),      "ESC off after reset");
    }
    return true;
}

FATP_TEST_CASE(stress_rejected_commands_do_not_corrupt_state)
{
    FullStack f;
    static constexpr const char* kWrongCmds[] = {
        "disarm", "takeoff", "land", "landing_complete",
        "disarm_after_landing", "emergency bad", "reset"
    };
    for (int i = 0; i < 20; ++i)
    {
        for (const char* c : kWrongCmds) (void)f.cmd.execute(c);
        FATP_ASSERT_TRUE(f.sm.isPreflight(), "f.sm.isPreflight()");
    }
    return true;
}

FATP_TEST_CASE(stress_telemetry_log_fills_and_caps)
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager      mgr{hub};
    drone::VehicleStateMachine   sm{mgr, hub};
    drone::TelemetryLog<16>      log{hub};
    drone::CommandParser<16>     cmd{mgr, sm, log};

    using namespace drone::subsystems;
    (void)mgr.enableSubsystem(kIMU);
    (void)mgr.enableSubsystem(kBarometer);
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
    FATP_ASSERT_FALSE(log.empty(), "log.empty()");
    return true;
}

} // namespace fat_p::testing::dronecore

namespace fat_p::testing
{

bool test_DroneCore()
{
    FATP_PRINT_HEADER(DRONE CORE INTEGRATION)

    TestRunner runner;

    // Graph-native power chain / profile tests
    FATP_RUN_TEST_NS(runner, dronecore, arm_brings_up_power_chain_via_graph);
    FATP_RUN_TEST_NS(runner, dronecore, disarm_tears_down_power_chain_via_graph);
    FATP_RUN_TEST_NS(runner, dronecore, disarm_after_landing_tears_down_power_chain_via_graph);
    FATP_RUN_TEST_NS(runner, dronecore, airborne_emergency_uses_emergency_land_profile);
    FATP_RUN_TEST_NS(runner, dronecore, airborne_emergency_reset_auto_cleans_power_chain_no_explicit_disable);
    FATP_RUN_TEST_NS(runner, dronecore, ground_emergency_message_says_stop);
    FATP_RUN_TEST_NS(runner, dronecore, ground_emergency_reset_restores_preflight);

    // Reserved feature guards
    FATP_RUN_TEST_NS(runner, dronecore, enable_emergency_stop_raw_blocked);
    FATP_RUN_TEST_NS(runner, dronecore, disable_emergency_stop_raw_blocked);
    FATP_RUN_TEST_NS(runner, dronecore, enable_armed_profile_raw_blocked);
    FATP_RUN_TEST_NS(runner, dronecore, enable_emergency_land_profile_raw_blocked);

    // Basic commands
    FATP_RUN_TEST_NS(runner, dronecore, command_unknown_returns_error);
    FATP_RUN_TEST_NS(runner, dronecore, command_empty_line_ok);
    FATP_RUN_TEST_NS(runner, dronecore, command_help_returns_text);
    FATP_RUN_TEST_NS(runner, dronecore, command_quit_sets_quit_flag);
    FATP_RUN_TEST_NS(runner, dronecore, command_enable_success);
    FATP_RUN_TEST_NS(runner, dronecore, command_enable_missing_arg);
    FATP_RUN_TEST_NS(runner, dronecore, command_disable_success);
    FATP_RUN_TEST_NS(runner, dronecore, command_status_shows_state);
    FATP_RUN_TEST_NS(runner, dronecore, command_arm_without_prereqs_fails);
    FATP_RUN_TEST_NS(runner, dronecore, command_arm_success);
    FATP_RUN_TEST_NS(runner, dronecore, command_full_flight_sequence);
    FATP_RUN_TEST_NS(runner, dronecore, command_disarm_after_landing);
    FATP_RUN_TEST_NS(runner, dronecore, command_emergency_and_reset);
    FATP_RUN_TEST_NS(runner, dronecore, command_log_after_events);
    FATP_RUN_TEST_NS(runner, dronecore, command_graph_returns_dot);
    FATP_RUN_TEST_NS(runner, dronecore, command_json_returns_json);
    FATP_RUN_TEST_NS(runner, dronecore, integration_case_insensitive_command);
    FATP_RUN_TEST_NS(runner, dronecore, integration_telemetry_captures_full_flight);

    // Adversarial
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_takeoff_from_preflight_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_disarm_from_flying_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_arm_when_already_armed_fails);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_command_with_leading_whitespace);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_enable_unknown_subsystem);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_very_long_subsystem_name);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_null_byte_in_command);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_takeoff_no_flight_mode_error_message);
    FATP_RUN_TEST_NS(runner, dronecore, adversarial_arm_blocked_while_emergency_stop_active);

    // Stress
    FATP_RUN_TEST_NS(runner, dronecore, stress_repeated_full_flight_via_commands);
    FATP_RUN_TEST_NS(runner, dronecore, stress_emergency_reset_via_commands);
    FATP_RUN_TEST_NS(runner, dronecore, stress_rejected_commands_do_not_corrupt_state);
    FATP_RUN_TEST_NS(runner, dronecore, stress_telemetry_log_fills_and_caps);

    return 0 == runner.print_summary();
}

} // namespace fat_p::testing

#ifdef ENABLE_TEST_APPLICATION
int main() { return fat_p::testing::test_DroneCore() ? 0 : 1; }
#endif
