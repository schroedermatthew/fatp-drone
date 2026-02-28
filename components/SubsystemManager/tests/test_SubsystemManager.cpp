/**
 * @file test_SubsystemManager.cpp
 * @brief Unit tests for SubsystemManager.h
 *
 * Tests cover: feature registration, dependency auto-enabling (Requires cascade),
 * mutual exclusion, implication cascade, conflict enforcement, arming validation,
 * and emergency stop conflicts.
 *
 * Key FeatureManager semantics reflected in tests:
 * - Requires: enabling A auto-enables all required features transitively.
 * - Implies:  enabling A also auto-enables implied features.
 * - MutuallyExclusive / Conflicts: enabling A fails if a conflicting feature is already on.
 */
/*
FATP_META:
  meta_version: 1
  component: SubsystemManager
  file_role: test
  path: components/SubsystemManager/tests/test_SubsystemManager.cpp
  namespace: fat_p::testing::subsystemmanager
  layer: Testing
  summary: Unit tests for SubsystemManager - dependency graph enforcement.
  api_stability: in_work
  related:
    headers:
      - include/drone/SubsystemManager.h
      - include/drone/DroneEvents.h
      - include/drone/Subsystems.h
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
#include <vector>

#include "DroneEvents.h"
#include "FatPTest.h"
#include "SubsystemManager.h"
#include "Subsystems.h"

namespace fat_p::testing::subsystemmanager
{

using namespace drone::subsystems;

struct Fixture
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager mgr{hub};
};

// ============================================================================
// Tests
// ============================================================================

FATP_TEST_CASE(initial_state_all_disabled)
{
    Fixture f;

    FATP_ASSERT_FALSE(f.mgr.isEnabled(kIMU),           "IMU should start disabled");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kGPS),           "GPS should start disabled");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kAutonomous),    "Autonomous should start disabled");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kEmergencyStop), "EmergencyStop should start disabled");
    FATP_ASSERT_TRUE(f.mgr.enabledSubsystems().empty(), "No subsystems should be enabled");
    return true;
}

FATP_TEST_CASE(enable_independent_sensor)
{
    Fixture f;

    auto res = f.mgr.enableSubsystem(kGPS);
    FATP_ASSERT_TRUE(res.has_value(), "Enable GPS should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS), "GPS should be enabled");
    return true;
}

FATP_TEST_CASE(disable_enabled_sensor)
{
    Fixture f;

    (void)f.mgr.enableSubsystem(kGPS);
    auto res = f.mgr.disableSubsystem(kGPS);
    FATP_ASSERT_TRUE(res.has_value(), "Disable GPS should succeed");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kGPS), "GPS should be disabled");
    return true;
}

FATP_TEST_CASE(requires_auto_enables_dependencies)
{
    // FeatureManager Requires = auto-enable cascade.
    // Enabling Stabilize auto-enables IMU + Barometer.
    Fixture f;

    auto res = f.mgr.enableSubsystem(kStabilize);
    FATP_ASSERT_TRUE(res.has_value(), "Enable Stabilize should succeed (auto-enables deps)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kStabilize), "Stabilize should be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU),       "IMU should be auto-enabled via Requires");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer), "Barometer should be auto-enabled via Requires");
    return true;
}

FATP_TEST_CASE(requires_chain_poshold_enables_sensors)
{
    // PosHold Requires IMU + Barometer + GPS — all auto-enabled.
    Fixture f;

    auto res = f.mgr.enableSubsystem(kPosHold);
    FATP_ASSERT_TRUE(res.has_value(), "Enable PosHold should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kPosHold),   "PosHold should be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU),       "IMU auto-enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer), "Barometer auto-enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS),       "GPS auto-enabled");
    return true;
}

FATP_TEST_CASE(autonomous_implies_collision_avoidance)
{
    // Enabling Autonomous should auto-enable CollisionAvoidance (Implies) and all sensors (Requires).
    Fixture f;

    (void)f.mgr.enableSubsystem(kDatalink); // Datalink has no deps, enable manually first
    auto res = f.mgr.enableSubsystem(kAutonomous);
    FATP_ASSERT_TRUE(res.has_value(), "Autonomous should succeed with Datalink enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kAutonomous),    "Autonomous should be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kCollisionAvoid),
                     "CollisionAvoidance should be auto-enabled via Implies");
    return true;
}

FATP_TEST_CASE(autonomous_requires_datalink)
{
    // Autonomous Requires Datalink. Datalink has no auto-enable source, so if
    // Datalink is not available and we try to enable Autonomous, it will auto-enable Datalink too.
    // (Datalink has no conflicting constraints so that should succeed.)
    Fixture f;

    auto res = f.mgr.enableSubsystem(kAutonomous);
    // Datalink has no deps of its own, so it will be auto-enabled successfully.
    FATP_ASSERT_TRUE(res.has_value(), "Autonomous should succeed (Datalink auto-enabled)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kDatalink), "Datalink should be auto-enabled");
    return true;
}

FATP_TEST_CASE(flight_modes_mutually_exclusive)
{
    Fixture f;

    auto res = f.mgr.enableSubsystem(kManual);
    FATP_ASSERT_TRUE(res.has_value(), "Manual should enable");

    // AltHold is MutuallyExclusive with Manual
    auto res2 = f.mgr.enableSubsystem(kAltHold);
    FATP_ASSERT_FALSE(res2.has_value(), "AltHold should be rejected while Manual is active");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kManual),  "Manual should still be enabled");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kAltHold), "AltHold should not be enabled");
    return true;
}

FATP_TEST_CASE(two_flight_modes_cannot_coexist)
{
    Fixture f;

    (void)f.mgr.enableSubsystem(kStabilize);
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kStabilize), "Stabilize should be on");

    auto res = f.mgr.enableSubsystem(kPosHold);
    FATP_ASSERT_FALSE(res.has_value(), "PosHold should be rejected while Stabilize is active");
    return true;
}

FATP_TEST_CASE(emergency_stop_preempts_active_flight_mode)
{
    // EmergencyStop now Preempts all flight modes.
    // Enabling EmergencyStop while a flight mode is active must:
    //   1. Succeed (Preempts is an authoritative shutdown, not a conflict block).
    //   2. Force-disable the active flight mode.
    //   3. Latch inhibit: re-enabling the flight mode while EmergencyStop is on must fail.
    Fixture f;

    // Pre-condition: Manual is active.
    auto arm = f.mgr.enableSubsystem(kManual);
    FATP_ASSERT_TRUE(arm.has_value(), "Manual must enable successfully as pre-condition");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kManual), "Manual should be enabled before estop");

    // EmergencyStop preempts: must succeed and Manual must be force-disabled.
    auto res = f.mgr.enableSubsystem(kEmergencyStop);
    FATP_ASSERT_TRUE(res.has_value(),                "EmergencyStop Preempts must succeed while Manual is active");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kEmergencyStop), "EmergencyStop should be enabled");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kManual),       "Manual should be force-disabled by Preempts cascade");

    // Latched inhibit: flight mode cannot re-enable while EmergencyStop is on.
    auto reEnable = f.mgr.enableSubsystem(kManual);
    FATP_ASSERT_FALSE(reEnable.has_value(), "Manual must not re-enable while EmergencyStop (Preempts) is active");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kManual), "Manual should remain disabled");
    return true;
}

FATP_TEST_CASE(emergency_stop_when_no_flight_mode)
{
    Fixture f;

    auto res = f.mgr.enableSubsystem(kEmergencyStop);
    FATP_ASSERT_TRUE(res.has_value(), "EmergencyStop should succeed with no active flight mode");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kEmergencyStop), "EmergencyStop should be enabled");
    return true;
}

FATP_TEST_CASE(disable_dependency_blocks_if_dependent_enabled)
{
    // After Stabilize is enabled (which auto-enables IMU), disabling IMU
    // should fail because Stabilize requires it and is still on.
    Fixture f;

    (void)f.mgr.enableSubsystem(kStabilize);
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kStabilize), "Stabilize should be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU),       "IMU should be auto-enabled");

    auto res = f.mgr.disableSubsystem(kIMU);
    FATP_ASSERT_FALSE(res.has_value(), "Disabling IMU should fail while Stabilize (which requires it) is active");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU), "IMU should still be enabled");
    return true;
}

FATP_TEST_CASE(validate_arming_readiness_missing_subsystems)
{
    Fixture f;

    auto res = f.mgr.validateArmingReadiness();
    FATP_ASSERT_FALSE(res.has_value(), "Arming readiness should fail with nothing enabled");
    return true;
}

FATP_TEST_CASE(validate_arming_readiness_full)
{
    Fixture f;

    // Enable all arm-required subsystems.
    // BatteryMonitor->ESC->MotorMix chain and RCReceiver are independent.
    // IMU and Barometer are independent sensors.
    (void)f.mgr.enableSubsystem(kIMU);
    (void)f.mgr.enableSubsystem(kBarometer);
    (void)f.mgr.enableSubsystem(kBatteryMonitor);
    (void)f.mgr.enableSubsystem(kESC);       // auto-enables BatteryMonitor (already on)
    (void)f.mgr.enableSubsystem(kMotorMix);  // auto-enables ESC (already on)
    (void)f.mgr.enableSubsystem(kRCReceiver);

    auto res = f.mgr.validateArmingReadiness();
    FATP_ASSERT_TRUE(res.has_value(), "Arming readiness should pass with all required subsystems");
    return true;
}

FATP_TEST_CASE(power_chain_auto_enable)
{
    // MotorMix Requires ESC Requires BatteryMonitor.
    // Enabling MotorMix should auto-enable ESC and BatteryMonitor.
    Fixture f;

    auto res = f.mgr.enableSubsystem(kMotorMix);
    FATP_ASSERT_TRUE(res.has_value(), "Enable MotorMix should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix),       "MotorMix should be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kESC),            "ESC should be auto-enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBatteryMonitor), "BatteryMonitor should be auto-enabled");
    return true;
}

FATP_TEST_CASE(active_flight_mode_query_empty)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.activeFlightMode().empty(), "No active flight mode initially");
    return true;
}

FATP_TEST_CASE(active_flight_mode_query_manual)
{
    Fixture f;

    (void)f.mgr.enableSubsystem(kManual);
    FATP_ASSERT_EQ(f.mgr.activeFlightMode(), std::string(kManual),
                   "Active flight mode should be Manual");
    return true;
}

FATP_TEST_CASE(subsystem_change_event_fired)
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager mgr{hub};

    std::vector<std::string> changedNames;
    std::vector<bool>        changedStates;

    auto conn = hub.onSubsystemChanged.connect(
        [&](std::string_view name, bool enabled)
        {
            changedNames.emplace_back(name);
            changedStates.push_back(enabled);
        });

    (void)mgr.enableSubsystem(kGPS);

    FATP_ASSERT_FALSE(changedNames.empty(), "onSubsystemChanged should have fired");

    bool found = false;
    for (std::size_t i = 0; i < changedNames.size(); ++i)
    {
        if (changedNames[i] == kGPS && changedStates[i] == true)
        {
            found = true;
        }
    }
    FATP_ASSERT_TRUE(found, "GPS enabled event should be present");
    return true;
}

FATP_TEST_CASE(json_output_contains_enabled_features)
{
    Fixture f;

    (void)f.mgr.enableSubsystem(kIMU);
    (void)f.mgr.enableSubsystem(kManual);

    const std::string json = f.mgr.toJson();
    FATP_ASSERT_FALSE(json.empty(), "JSON output should not be empty");
    FATP_ASSERT_CONTAINS(json, "IMU",    "JSON should contain IMU");
    FATP_ASSERT_CONTAINS(json, "Manual", "JSON should contain Manual");
    return true;
}

FATP_TEST_CASE(dot_export_contains_digraph)
{
    Fixture f;

    const std::string dot = f.mgr.exportDependencyGraph();
    FATP_ASSERT_FALSE(dot.empty(),         "DOT export should not be empty");
    FATP_ASSERT_CONTAINS(dot, "digraph",   "DOT output should contain 'digraph'");
    return true;
}

} // namespace fat_p::testing::subsystemmanager

namespace fat_p::testing
{

bool test_SubsystemManager()
{
    FATP_PRINT_HEADER(SUBSYSTEM MANAGER)

    TestRunner runner;

    FATP_RUN_TEST_NS(runner, subsystemmanager, initial_state_all_disabled);
    FATP_RUN_TEST_NS(runner, subsystemmanager, enable_independent_sensor);
    FATP_RUN_TEST_NS(runner, subsystemmanager, disable_enabled_sensor);
    FATP_RUN_TEST_NS(runner, subsystemmanager, requires_auto_enables_dependencies);
    FATP_RUN_TEST_NS(runner, subsystemmanager, requires_chain_poshold_enables_sensors);
    FATP_RUN_TEST_NS(runner, subsystemmanager, autonomous_implies_collision_avoidance);
    FATP_RUN_TEST_NS(runner, subsystemmanager, autonomous_requires_datalink);
    FATP_RUN_TEST_NS(runner, subsystemmanager, flight_modes_mutually_exclusive);
    FATP_RUN_TEST_NS(runner, subsystemmanager, two_flight_modes_cannot_coexist);
    FATP_RUN_TEST_NS(runner, subsystemmanager, emergency_stop_preempts_active_flight_mode);
    FATP_RUN_TEST_NS(runner, subsystemmanager, emergency_stop_when_no_flight_mode);
    FATP_RUN_TEST_NS(runner, subsystemmanager, disable_dependency_blocks_if_dependent_enabled);
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_arming_readiness_missing_subsystems);
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_arming_readiness_full);
    FATP_RUN_TEST_NS(runner, subsystemmanager, power_chain_auto_enable);
    FATP_RUN_TEST_NS(runner, subsystemmanager, active_flight_mode_query_empty);
    FATP_RUN_TEST_NS(runner, subsystemmanager, active_flight_mode_query_manual);
    FATP_RUN_TEST_NS(runner, subsystemmanager, subsystem_change_event_fired);
    FATP_RUN_TEST_NS(runner, subsystemmanager, json_output_contains_enabled_features);
    FATP_RUN_TEST_NS(runner, subsystemmanager, dot_export_contains_digraph);

    return 0 == runner.print_summary();
}

} // namespace fat_p::testing

#ifdef ENABLE_TEST_APPLICATION
int main()
{
    return fat_p::testing::test_SubsystemManager() ? 0 : 1;
}
#endif
