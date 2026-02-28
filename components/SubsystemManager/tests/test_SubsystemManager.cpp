/**
 * @file test_SubsystemManager.cpp
 * @brief Unit tests for SubsystemManager.h
 *
 * Tests cover: feature registration, dependency auto-enabling (Requires cascade),
 * mutual exclusion, implication cascade, conflict enforcement, arming validation,
 * emergency stop conflicts, independent sensors, RTL, Failsafe, adversarial inputs,
 * EmergencyStop latch coverage across all modes, and stress/fuzz operations.
 *
 * Key FeatureManager semantics reflected in tests:
 * - Requires: enabling A auto-enables all required features transitively.
 * - Implies:  enabling A also auto-enables implied features.
 * - MutuallyExclusive / Conflicts: enabling A fails if a conflicting feature is already on.
 * - Preempts: enabling A force-disables B, cascades reverse deps, and latches inhibit.
 */
/*
FATP_META:
  meta_version: 1
  component: SubsystemManager
  file_role: test
  path: components/SubsystemManager/tests/test_SubsystemManager.cpp
  namespace: fat_p::testing::subsystemmanager
  layer: Testing
  summary: Unit tests for SubsystemManager - dependency graph, adversarial inputs, stress.
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
#include <random>
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
// Basic / Happy Path
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
    Fixture f;
    auto res = f.mgr.enableSubsystem(kAutonomous);
    FATP_ASSERT_TRUE(res.has_value(), "Autonomous should succeed (all deps auto-enabled)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kAutonomous),    "Autonomous should be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kCollisionAvoid),"CollisionAvoidance auto-enabled via Implies");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU),           "IMU auto-enabled via Requires");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer),     "Barometer auto-enabled via Requires");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS),           "GPS auto-enabled via Requires");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kDatalink),      "Datalink auto-enabled via Requires");
    return true;
}

FATP_TEST_CASE(autonomous_requires_datalink)
{
    Fixture f;
    auto res = f.mgr.enableSubsystem(kAutonomous);
    FATP_ASSERT_TRUE(res.has_value(), "Autonomous should succeed (Datalink auto-enabled)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kDatalink), "Datalink should be auto-enabled");
    return true;
}

FATP_TEST_CASE(flight_modes_mutually_exclusive)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    auto res2 = f.mgr.enableSubsystem(kAltHold);
    FATP_ASSERT_FALSE(res2.has_value(), "AltHold should be rejected while Manual is active");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kManual),   "Manual should still be enabled");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kAltHold), "AltHold should not be enabled");
    return true;
}

FATP_TEST_CASE(two_flight_modes_cannot_coexist)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kStabilize);
    auto res = f.mgr.enableSubsystem(kPosHold);
    FATP_ASSERT_FALSE(res.has_value(), "PosHold should be rejected while Stabilize is active");
    return true;
}

FATP_TEST_CASE(emergency_stop_preempts_active_flight_mode)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    auto res = f.mgr.enableSubsystem(kEmergencyStop);
    FATP_ASSERT_TRUE(res.has_value(),                 "EmergencyStop Preempts must succeed while Manual is active");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kEmergencyStop), "EmergencyStop should be enabled");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kManual),       "Manual should be force-disabled by Preempts cascade");
    auto reEnable = f.mgr.enableSubsystem(kManual);
    FATP_ASSERT_FALSE(reEnable.has_value(), "Manual must not re-enable while EmergencyStop is active");
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
    Fixture f;
    (void)f.mgr.enableSubsystem(kStabilize);
    auto res = f.mgr.disableSubsystem(kIMU);
    FATP_ASSERT_FALSE(res.has_value(), "Disabling IMU should fail while Stabilize (which requires it) is active");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU), "IMU should still be enabled");
    return true;
}

FATP_TEST_CASE(validate_arming_readiness_missing_subsystems)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.validateArmingReadiness().has_value(),
                      "Arming readiness should fail with nothing enabled");
    return true;
}

FATP_TEST_CASE(validate_arming_readiness_full)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kIMU);
    (void)f.mgr.enableSubsystem(kBarometer);
    (void)f.mgr.enableSubsystem(kBatteryMonitor);
    (void)f.mgr.enableSubsystem(kESC);
    (void)f.mgr.enableSubsystem(kMotorMix);
    (void)f.mgr.enableSubsystem(kRCReceiver);
    FATP_ASSERT_TRUE(f.mgr.validateArmingReadiness().has_value(),
                     "Arming readiness should pass with all required subsystems");
    return true;
}

FATP_TEST_CASE(power_chain_auto_enable)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kMotorMix);
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
        if (changedNames[i] == kGPS && changedStates[i]) { found = true; }
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
    FATP_ASSERT_FALSE(dot.empty(),       "DOT export should not be empty");
    FATP_ASSERT_CONTAINS(dot, "digraph", "DOT output should contain 'digraph'");
    return true;
}

// ============================================================================
// Previously untested subsystems
// ============================================================================

FATP_TEST_CASE(rtl_auto_enables_imu_barometer_gps)
{
    // RTL Requires IMU, Barometer, GPS (flat deps — not chained through AltHold).
    Fixture f;
    auto res = f.mgr.enableSubsystem(kRTL);
    FATP_ASSERT_TRUE(res.has_value(), "Enable RTL should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kRTL),       "RTL should be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU),       "IMU auto-enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer), "Barometer auto-enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS),       "GPS auto-enabled");
    // RTL does NOT chain through AltHold — they are MutuallyExclusive
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kAltHold),  "AltHold must NOT be auto-enabled by RTL");
    return true;
}

FATP_TEST_CASE(rtl_mutually_exclusive_with_other_modes)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kRTL);
    FATP_ASSERT_FALSE(f.mgr.enableSubsystem(kManual).has_value(),
                      "Manual should be rejected while RTL is active");
    return true;
}

FATP_TEST_CASE(althold_auto_enables_imu_barometer_not_stabilize)
{
    // AltHold Requires IMU + Barometer directly — not through Stabilize.
    // They are MutuallyExclusive so Stabilize cannot be auto-enabled.
    Fixture f;
    auto res = f.mgr.enableSubsystem(kAltHold);
    FATP_ASSERT_TRUE(res.has_value(), "AltHold should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU),       "IMU auto-enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer), "Barometer auto-enabled");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kStabilize),"Stabilize must NOT be auto-enabled (MutuallyExclusive)");
    return true;
}

FATP_TEST_CASE(failsafe_auto_enables_battery_monitor_and_rcreceiver)
{
    Fixture f;
    auto res = f.mgr.enableSubsystem(kFailsafe);
    FATP_ASSERT_TRUE(res.has_value(), "Enable Failsafe should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kFailsafe),       "Failsafe should be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBatteryMonitor), "BatteryMonitor auto-enabled via Requires");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kRCReceiver),     "RCReceiver auto-enabled via Requires");
    return true;
}

FATP_TEST_CASE(geofence_is_independent)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kGeofence).has_value(), "Geofence should enable");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGeofence), "Geofence should be enabled");
    FATP_ASSERT_TRUE(f.mgr.disableSubsystem(kGeofence).has_value(), "Geofence should disable");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kGeofence), "Geofence should be disabled");
    return true;
}

FATP_TEST_CASE(compass_optical_flow_lidar_are_independent)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kCompass).has_value(),     "Compass should enable");
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kOpticalFlow).has_value(), "OpticalFlow should enable");
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kLidar).has_value(),       "Lidar should enable");
    FATP_ASSERT_TRUE(f.mgr.disableSubsystem(kCompass).has_value(),    "Compass should disable");
    FATP_ASSERT_TRUE(f.mgr.disableSubsystem(kOpticalFlow).has_value(),"OpticalFlow should disable");
    FATP_ASSERT_TRUE(f.mgr.disableSubsystem(kLidar).has_value(),      "Lidar should disable");
    return true;
}

FATP_TEST_CASE(telemetry_is_independent)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kTelemetry).has_value(), "Telemetry should enable");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kTelemetry), "Telemetry should be enabled");
    return true;
}

// ============================================================================
// Adversarial — hostile / unexpected inputs
// ============================================================================

FATP_TEST_CASE(adversarial_enable_unknown_subsystem)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.enableSubsystem("NotASubsystem").has_value(),
                      "Enabling unknown subsystem should fail");
    return true;
}

FATP_TEST_CASE(adversarial_disable_unknown_subsystem)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.disableSubsystem("GhostSubsystem").has_value(),
                      "Disabling unknown subsystem should fail");
    return true;
}

FATP_TEST_CASE(adversarial_enable_empty_name)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.enableSubsystem("").has_value(),
                      "Enabling empty-string name should fail");
    return true;
}

FATP_TEST_CASE(adversarial_disable_not_enabled_subsystem)
{
    // FeatureManager treats disabling an already-disabled feature as a no-op success.
    // The critical postcondition is that state remains consistent (GPS stays disabled).
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kGPS), "GPS should start disabled");
    (void)f.mgr.disableSubsystem(kGPS); // result is implementation-defined (no-op or error)
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kGPS), "GPS must remain disabled");
    return true;
}

FATP_TEST_CASE(adversarial_enable_same_subsystem_twice)
{
    // Idempotent or clean failure — state must remain consistent.
    Fixture f;
    (void)f.mgr.enableSubsystem(kGPS);
    (void)f.mgr.enableSubsystem(kGPS); // second call — result is implementation-defined
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS), "GPS must still be enabled after second enable");
    return true;
}

FATP_TEST_CASE(adversarial_cascading_disable_blocked)
{
    // Enable PosHold; all its deps must refuse to be individually disabled.
    Fixture f;
    (void)f.mgr.enableSubsystem(kPosHold);
    FATP_ASSERT_FALSE(f.mgr.disableSubsystem(kIMU).has_value(),
                      "Disabling IMU while PosHold is active should fail");
    FATP_ASSERT_FALSE(f.mgr.disableSubsystem(kBarometer).has_value(),
                      "Disabling Barometer while PosHold is active should fail");
    FATP_ASSERT_FALSE(f.mgr.disableSubsystem(kGPS).has_value(),
                      "Disabling GPS while PosHold is active should fail");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU),       "IMU should still be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer), "Barometer should still be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS),       "GPS should still be enabled");
    return true;
}

FATP_TEST_CASE(adversarial_all_flight_modes_rejected_with_one_active)
{
    // With Manual active, every other mode must be rejected.
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);

    static constexpr const char* kOtherModes[] = {
        kStabilize, kAltHold, kPosHold, kAutonomous, kRTL
    };
    for (const char* mode : kOtherModes)
    {
        FATP_ASSERT_FALSE(f.mgr.enableSubsystem(mode).has_value(),
                          (std::string("Mode ") + mode + " should be rejected with Manual active").c_str());
    }
    FATP_ASSERT_EQ(f.mgr.activeFlightMode(), std::string(kManual),
                   "Manual should remain the active mode");
    return true;
}

FATP_TEST_CASE(adversarial_emergency_stop_latch_covers_all_modes)
{
    // EmergencyStop inhibit latch must cover ALL 6 modes, not just the active one.
    Fixture f;
    (void)f.mgr.enableSubsystem(kStabilize);
    (void)f.mgr.enableSubsystem(kEmergencyStop);

    FATP_ASSERT_TRUE(f.mgr.isEnabled(kEmergencyStop), "EmergencyStop should be on");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kStabilize),    "Stabilize force-disabled");

    static constexpr const char* kAllModes[] = {
        kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL
    };
    for (const char* mode : kAllModes)
    {
        FATP_ASSERT_FALSE(f.mgr.enableSubsystem(mode).has_value(),
                          (std::string("Mode ") + mode + " must be inhibited while EmergencyStop is latched").c_str());
    }
    return true;
}

FATP_TEST_CASE(adversarial_validate_arming_readiness_each_missing_subsystem)
{
    // Remove each arm-required subsystem in turn and confirm readiness fails.
    // Critical: ESC Requires BatteryMonitor, MotorMix Requires ESC.
    // Skipping BatteryMonitor must also skip ESC and MotorMix, otherwise
    // enabling ESC will auto-enable BatteryMonitor via Requires cascade.
    // Similarly, skipping ESC must also skip MotorMix.
    static constexpr const char* kArmRequired[] = {
        kIMU, kBarometer, kBatteryMonitor, kESC, kMotorMix, kRCReceiver
    };

    // For each missing subsystem, record which additional subsystems must
    // also be withheld to prevent the cascade from re-introducing it.
    struct TestCase { const char* missing; const char* alsoSkip[2]; };
    static constexpr TestCase kCases[] = {
        { kIMU,            { nullptr,  nullptr  } },
        { kBarometer,      { nullptr,  nullptr  } },
        { kBatteryMonitor, { kESC,     kMotorMix } },  // ESC auto-enables BatteryMonitor
        { kESC,            { kMotorMix, nullptr  } },  // MotorMix auto-enables ESC
        { kMotorMix,       { nullptr,  nullptr  } },
        { kRCReceiver,     { nullptr,  nullptr  } },
    };

    for (const auto& tc : kCases)
    {
        drone::events::DroneEventHub hub;
        drone::SubsystemManager mgr{hub};

        for (const char* sub : kArmRequired)
        {
            if (std::string_view(sub) == std::string_view(tc.missing)) { continue; }
            if (tc.alsoSkip[0] && std::string_view(sub) == std::string_view(tc.alsoSkip[0])) { continue; }
            if (tc.alsoSkip[1] && std::string_view(sub) == std::string_view(tc.alsoSkip[1])) { continue; }
            (void)mgr.enableSubsystem(sub);
        }

        FATP_ASSERT_FALSE(mgr.validateArmingReadiness().has_value(),
                          (std::string("Arming should fail when ") + tc.missing + " is missing").c_str());
    }
    return true;
}

FATP_TEST_CASE(adversarial_error_event_fired_on_conflict)
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager mgr{hub};
    std::vector<std::string> errored;
    auto conn = hub.onSubsystemError.connect(
        [&](std::string_view name, std::string_view)
        { errored.emplace_back(name); });

    (void)mgr.enableSubsystem(kManual);
    (void)mgr.enableSubsystem(kStabilize); // MutuallyExclusive conflict

    FATP_ASSERT_FALSE(errored.empty(), "onSubsystemError should fire on constraint violation");
    return true;
}

// ============================================================================
// Stress / fuzz
// ============================================================================

FATP_TEST_CASE(stress_repeated_enable_disable_independent_sensors)
{
    Fixture f;
    static constexpr const char* kSensors[] = {
        kCompass, kOpticalFlow, kLidar, kTelemetry, kGeofence
    };
    for (int round = 0; round < 50; ++round)
    {
        for (const char* s : kSensors)
        {
            (void)f.mgr.enableSubsystem(s);
            FATP_ASSERT_TRUE(f.mgr.isEnabled(s),
                             (std::string(s) + " should be on after enable").c_str());
            (void)f.mgr.disableSubsystem(s);
            FATP_ASSERT_FALSE(f.mgr.isEnabled(s),
                              (std::string(s) + " should be off after disable").c_str());
        }
    }
    return true;
}

FATP_TEST_CASE(stress_random_subsystem_operations)
{
    // Random enable/disable against constraint-free subsystems, verified against
    // a reference bool array at each step.
    drone::events::DroneEventHub hub;
    drone::SubsystemManager mgr{hub};

    static constexpr const char* kFree[] = {
        kCompass, kOpticalFlow, kLidar, kTelemetry, kGeofence, kGPS
    };
    constexpr int kN = 6;
    bool enabled[kN] = {};

    std::mt19937 rng(0xFAB1C0DE);
    std::uniform_int_distribution<int> idxDist(0, kN - 1);
    std::uniform_int_distribution<int> opDist(0, 1);

    for (int i = 0; i < 300; ++i)
    {
        int idx = idxDist(rng);
        const char* sub = kFree[idx];

        if (opDist(rng) == 0)
        {
            if (mgr.enableSubsystem(sub).has_value()) { enabled[idx] = true; }
        }
        else
        {
            if (mgr.disableSubsystem(sub).has_value()) { enabled[idx] = false; }
        }

        for (int j = 0; j < kN; ++j)
        {
            FATP_ASSERT_EQ(mgr.isEnabled(kFree[j]), enabled[j],
                           "Reference must match actual state after random op");
        }
    }
    return true;
}

FATP_TEST_CASE(stress_flight_mode_cycle)
{
    // Cycle through flight modes by enabling then disabling each in turn.
    // Sensors must also be freed between cycles.
    Fixture f;
    static constexpr const char* kModes[] = {
        kManual, kStabilize, kAltHold, kPosHold, kRTL
    };
    for (int cycle = 0; cycle < 5; ++cycle)
    {
        for (const char* mode : kModes)
        {
            FATP_ASSERT_TRUE(f.mgr.enableSubsystem(mode).has_value(),
                             (std::string("Should enable ") + mode).c_str());
            FATP_ASSERT_EQ(f.mgr.activeFlightMode(), std::string(mode),
                           "Active mode should match just-enabled mode");

            FATP_ASSERT_TRUE(f.mgr.disableSubsystem(mode).has_value(),
                             (std::string("Should disable ") + mode).c_str());
            FATP_ASSERT_TRUE(f.mgr.activeFlightMode().empty(),
                             "No active mode after disable");

            // Free auto-enabled sensors so next mode starts clean
            (void)f.mgr.disableSubsystem(kIMU);
            (void)f.mgr.disableSubsystem(kBarometer);
            (void)f.mgr.disableSubsystem(kGPS);
        }
    }
    return true;
}

} // namespace fat_p::testing::subsystemmanager

namespace fat_p::testing
{

bool test_SubsystemManager()
{
    FATP_PRINT_HEADER(SUBSYSTEM MANAGER)

    TestRunner runner;

    // Basic / happy path
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

    // Previously untested subsystems
    FATP_RUN_TEST_NS(runner, subsystemmanager, rtl_auto_enables_imu_barometer_gps);
    FATP_RUN_TEST_NS(runner, subsystemmanager, rtl_mutually_exclusive_with_other_modes);
    FATP_RUN_TEST_NS(runner, subsystemmanager, althold_auto_enables_imu_barometer_not_stabilize);
    FATP_RUN_TEST_NS(runner, subsystemmanager, failsafe_auto_enables_battery_monitor_and_rcreceiver);
    FATP_RUN_TEST_NS(runner, subsystemmanager, geofence_is_independent);
    FATP_RUN_TEST_NS(runner, subsystemmanager, compass_optical_flow_lidar_are_independent);
    FATP_RUN_TEST_NS(runner, subsystemmanager, telemetry_is_independent);

    // Adversarial
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_enable_unknown_subsystem);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_disable_unknown_subsystem);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_enable_empty_name);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_disable_not_enabled_subsystem);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_enable_same_subsystem_twice);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_cascading_disable_blocked);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_all_flight_modes_rejected_with_one_active);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_emergency_stop_latch_covers_all_modes);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_validate_arming_readiness_each_missing_subsystem);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_error_event_fired_on_conflict);

    // Stress / fuzz
    FATP_RUN_TEST_NS(runner, subsystemmanager, stress_repeated_enable_disable_independent_sensors);
    FATP_RUN_TEST_NS(runner, subsystemmanager, stress_random_subsystem_operations);
    FATP_RUN_TEST_NS(runner, subsystemmanager, stress_flight_mode_cycle);

    return 0 == runner.print_summary();
}

} // namespace fat_p::testing

#ifdef ENABLE_TEST_APPLICATION
int main()
{
    return fat_p::testing::test_SubsystemManager() ? 0 : 1;
}
#endif
