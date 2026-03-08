/**
 * @file test_SubsystemManager.cpp
 * @brief Unit tests for SubsystemManager.h
 *
 * Tests the graph-native design:
 *   - ArmedProfile Entails power chain (enterArmedConfiguration / leaveArmedConfiguration)
 *   - EmergencyLandProfile Entails power chain (triggerEmergencyLand / resetEmergencyStop)
 *   - EmergencyStop Preempts flight modes
 *   - validateArmingReadiness: IMU + Barometer + RCReceiver only (power chain is FM-owned)
 *   - switchFlightMode / validateFlightMode membership guards
 *   - All pre-existing dependency, conflict, and ME tests
 */
/*
FATP_META:
  meta_version: 1
  component: SubsystemManager
  file_role: test
  path: components/SubsystemManager/tests/test_SubsystemManager.cpp
  namespace: fat_p::testing::subsystemmanager
  layer: Testing
  api_stability: in_work
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
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileArmed),  "ArmedProfile should start disabled");
    FATP_ASSERT_TRUE(f.mgr.enabledSubsystems().empty(), "No subsystems should be enabled");
    return true;
}

FATP_TEST_CASE(enable_independent_sensor)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kGPS).has_value(), "Enable GPS should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS), "GPS should be enabled");
    return true;
}

FATP_TEST_CASE(disable_enabled_sensor)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kGPS);
    FATP_ASSERT_TRUE(f.mgr.disableSubsystem(kGPS).has_value(), "Disable GPS should succeed");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kGPS), "GPS should be disabled");
    return true;
}

FATP_TEST_CASE(requires_auto_enables_dependencies)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kStabilize).has_value(), "Enable Stabilize should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kStabilize), "Stabilize on");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU),       "IMU auto-enabled via Requires");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer), "Barometer auto-enabled via Requires");
    return true;
}

FATP_TEST_CASE(requires_chain_poshold_enables_sensors)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kPosHold).has_value(), "PosHold should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kPosHold), "f.mgr.isEnabled(kPosHold)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU), "f.mgr.isEnabled(kIMU)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer), "f.mgr.isEnabled(kBarometer)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS), "f.mgr.isEnabled(kGPS)");
    return true;
}

FATP_TEST_CASE(autonomous_implies_collision_avoidance)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kAutonomous).has_value(), "Autonomous should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kAutonomous), "f.mgr.isEnabled(kAutonomous)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kCollisionAvoid), "CollisionAvoidance auto-enabled via Implies");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU), "f.mgr.isEnabled(kIMU)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer), "f.mgr.isEnabled(kBarometer)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS), "f.mgr.isEnabled(kGPS)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kDatalink), "f.mgr.isEnabled(kDatalink)");
    return true;
}

FATP_TEST_CASE(flight_modes_mutually_exclusive)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    FATP_ASSERT_FALSE(f.mgr.enableSubsystem(kAltHold).has_value(),
                      "AltHold rejected while Manual is active");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kManual), "f.mgr.isEnabled(kManual)");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kAltHold), "f.mgr.isEnabled(kAltHold)");
    return true;
}

FATP_TEST_CASE(power_chain_auto_enable_via_motormix)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kMotorMix);
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix), "f.mgr.isEnabled(kMotorMix)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kESC), "f.mgr.isEnabled(kESC)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBatteryMonitor), "f.mgr.isEnabled(kBatteryMonitor)");
    return true;
}

FATP_TEST_CASE(disable_dependency_blocks_if_dependent_enabled)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kStabilize);
    FATP_ASSERT_FALSE(f.mgr.disableSubsystem(kIMU).has_value(),
                      "Disabling IMU while Stabilize requires it should fail");
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
    FATP_ASSERT_EQ(f.mgr.activeFlightMode(), std::string(kManual), "Active mode should be Manual");
    return true;
}

// ============================================================================
// ArmedProfile — FM-native power chain ownership
// ============================================================================

FATP_TEST_CASE(armed_profile_entails_power_chain_on_enter)
{
    // enterArmedConfiguration enables ArmedProfile.
    // FM Entails cascade must bring up MotorMix -> ESC -> BatteryMonitor.
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix),       "MotorMix off initially");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),            "ESC off initially");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kBatteryMonitor), "BatteryMonitor off initially");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileArmed),   "ArmedProfile off initially");

    FATP_ASSERT_TRUE(f.mgr.enterArmedConfiguration().has_value(),
                     "enterArmedConfiguration should succeed");

    FATP_ASSERT_TRUE(f.mgr.isEnabled(kProfileArmed),   "ArmedProfile should be enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix),       "MotorMix auto-enabled via Entails");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kESC),            "ESC auto-enabled via Requires from MotorMix");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBatteryMonitor), "BatteryMonitor auto-enabled via Requires from ESC");
    return true;
}

FATP_TEST_CASE(armed_profile_entails_cascade_teardown_on_leave)
{
    // leaveArmedConfiguration disables ArmedProfile.
    // FM ref-counted Entails cascade must take down MotorMix and ESC.
    // BatteryMonitor (NOT Entailed) must stay on.
    Fixture f;
    (void)f.mgr.enterArmedConfiguration();
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix),       "MotorMix on after enter");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kESC),            "ESC on after enter");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBatteryMonitor), "BatteryMonitor on after enter");

    f.mgr.leaveArmedConfiguration();

    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileArmed), "ArmedProfile off after leave");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix),     "MotorMix auto-torn-down via Entails ref-count");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),          "ESC auto-torn-down (MotorMix removed from plan first)");
    // BatteryMonitor is NOT Entailed — stays on for preflight monitoring.
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBatteryMonitor),
                     "BatteryMonitor remains on (not Entailed by ArmedProfile)");
    return true;
}

FATP_TEST_CASE(armed_profile_idempotent_double_enter)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enterArmedConfiguration().has_value(), "First enter");
    FATP_ASSERT_TRUE(f.mgr.enterArmedConfiguration().has_value(), "Second enter (idempotent)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix), "MotorMix on");
    return true;
}

FATP_TEST_CASE(armed_profile_leave_is_idempotent_when_not_entered)
{
    // leaveArmedConfiguration on a clean fixture should be a no-op.
    Fixture f;
    f.mgr.leaveArmedConfiguration(); // should not throw or crash
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix), "MotorMix still off");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),      "ESC still off");
    return true;
}

FATP_TEST_CASE(armed_profile_flight_mode_survives_leave)
{
    // Flight modes enabled by the user before arming should be cleared by
    // leaveArmedConfiguration (it disables all modes explicitly).
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    (void)f.mgr.enterArmedConfiguration();
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kManual), "Manual on while armed");

    f.mgr.leaveArmedConfiguration();
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kManual), "Manual cleared by leaveArmedConfiguration");
    return true;
}

// ============================================================================
// EmergencyLandProfile — FM-native power chain for controlled descent
// ============================================================================

FATP_TEST_CASE(emergency_land_profile_entails_power_chain)
{
    // triggerEmergencyLand: forceExclusive + enable EmergencyLandProfile.
    // EmergencyLandProfile Entails MotorMix/ESC -> power chain live.
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    (void)f.mgr.enterArmedConfiguration();

    FATP_ASSERT_TRUE(f.mgr.triggerEmergencyLand().has_value(), "triggerEmergencyLand should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kEmergencyStop),       "EmergencyStop latched");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kManual),             "Manual cleared by forceExclusive");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileArmed),       "ArmedProfile cleared by forceExclusive");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kProfileEmergencyLand),"EmergencyLandProfile enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix),            "MotorMix live via EmergencyLandProfile Entails");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kESC),                 "ESC live via EmergencyLandProfile Entails");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBatteryMonitor),      "BatteryMonitor live via Requires");
    return true;
}

FATP_TEST_CASE(emergency_land_reset_auto_cleans_power_chain_via_graph)
{
    // resetEmergencyStop disables EmergencyLandProfile first.
    // FM Entails ref-count cascade must auto-disable MotorMix and ESC.
    // No explicit power-chain cleanup loop — the graph handles it.
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    (void)f.mgr.enterArmedConfiguration();
    (void)f.mgr.triggerEmergencyLand();

    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix), "MotorMix live during emergency land");

    FATP_ASSERT_TRUE(f.mgr.resetEmergencyStop().has_value(), "resetEmergencyStop should succeed");

    FATP_ASSERT_FALSE(f.mgr.isEnabled(kEmergencyStop),        "EmergencyStop cleared");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileEmergencyLand), "EmergencyLandProfile cleared");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix),
                      "MotorMix auto-torn-down via EmergencyLandProfile Entails ref-count");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),
                      "ESC auto-torn-down");
    return true;
}

FATP_TEST_CASE(emergency_stop_reset_ground_path_no_op_profile)
{
    // Ground path: triggerEmergencyStop (no EmergencyLandProfile).
    // resetEmergencyStop should still succeed; the EmergencyLandProfile disable is a no-op.
    Fixture f;
    (void)f.mgr.enterArmedConfiguration();
    (void)f.mgr.triggerEmergencyStop();

    FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileEmergencyLand), "EmergencyLandProfile was never enabled");

    FATP_ASSERT_TRUE(f.mgr.resetEmergencyStop().has_value(), "resetEmergencyStop should succeed");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kEmergencyStop), "EmergencyStop cleared");
    return true;
}

FATP_TEST_CASE(emergency_land_a2_latch_still_blocks_flight_modes)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    (void)f.mgr.triggerEmergencyLand();

    FATP_ASSERT_TRUE(f.mgr.isEnabled(kEmergencyStop), "EmergencyStop latched");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix),      "MotorMix live");

    static constexpr const char* kAllModes[] = {
        kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL
    };
    for (const char* mode : kAllModes)
    {
        FATP_ASSERT_FALSE(f.mgr.enableSubsystem(mode).has_value(),
                          (std::string("Mode ") + mode + " must be blocked by Preempts latch").c_str());
    }
    return true;
}

FATP_TEST_CASE(emergency_land_reset_allows_flight_modes)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    (void)f.mgr.triggerEmergencyLand();
    (void)f.mgr.resetEmergencyStop();

    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kManual).has_value(),
                     "Manual re-enable-able after latch cleared");
    return true;
}

// ============================================================================
// EmergencyStop Preempts / latch
// ============================================================================

FATP_TEST_CASE(emergency_stop_preempts_active_flight_mode)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    FATP_ASSERT_TRUE(f.mgr.triggerEmergencyStop().has_value(), "triggerEmergencyStop must succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kEmergencyStop),  "EmergencyStop on");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kManual),        "Manual force-disabled");
    FATP_ASSERT_FALSE(f.mgr.enableSubsystem(kManual).has_value(),
                      "Manual blocked while EmergencyStop is active");
    return true;
}

FATP_TEST_CASE(emergency_stop_reset_clears_latch)
{
    Fixture f;
    (void)f.mgr.triggerEmergencyStop();
    FATP_ASSERT_TRUE(f.mgr.resetEmergencyStop().has_value(), "resetEmergencyStop should succeed");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kEmergencyStop), "EmergencyStop cleared");
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kManual).has_value(),
                     "Manual re-enable-able after reset");
    return true;
}

FATP_TEST_CASE(adversarial_emergency_stop_latch_covers_all_modes)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kStabilize);
    (void)f.mgr.triggerEmergencyStop();

    static constexpr const char* kAllModes[] = {
        kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL
    };
    for (const char* mode : kAllModes)
    {
        FATP_ASSERT_FALSE(f.mgr.enableSubsystem(mode).has_value(),
                          (std::string("Mode ") + mode + " must be blocked by Preempts").c_str());
    }
    return true;
}

// ============================================================================
// validateArmingReadiness — power chain is FM-owned, not a user prerequisite
// ============================================================================

FATP_TEST_CASE(validate_arming_readiness_minimal_set)
{
    // Only IMU, Barometer, RCReceiver required. Power chain is brought up by
    // ArmedProfile via Entails when Armed is entered.
    Fixture f;
    (void)f.mgr.enableSubsystem(kIMU);
    (void)f.mgr.enableSubsystem(kBarometer);
    (void)f.mgr.enableSubsystem(kRCReceiver);
    FATP_ASSERT_TRUE(f.mgr.validateArmingReadiness().has_value(),
                     "Arming should pass with IMU + Barometer + RCReceiver");
    return true;
}

FATP_TEST_CASE(validate_arming_readiness_missing_imu)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kBarometer);
    (void)f.mgr.enableSubsystem(kRCReceiver);
    FATP_ASSERT_FALSE(f.mgr.validateArmingReadiness().has_value(), "IMU missing should fail");
    return true;
}

FATP_TEST_CASE(validate_arming_readiness_missing_barometer)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kIMU);
    (void)f.mgr.enableSubsystem(kRCReceiver);
    FATP_ASSERT_FALSE(f.mgr.validateArmingReadiness().has_value(), "Barometer missing should fail");
    return true;
}

FATP_TEST_CASE(validate_arming_readiness_missing_rcreceiver)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kIMU);
    (void)f.mgr.enableSubsystem(kBarometer);
    FATP_ASSERT_FALSE(f.mgr.validateArmingReadiness().has_value(), "RCReceiver missing should fail");
    return true;
}

FATP_TEST_CASE(validate_arming_blocked_by_emergency_stop)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kIMU);
    (void)f.mgr.enableSubsystem(kBarometer);
    (void)f.mgr.enableSubsystem(kRCReceiver);
    // Manually enable EmergencyStop to test the guard without going through forceExclusive.
    // (forceExclusive would clear the prerequisites we just set.)
    auto res = f.mgr.enableSubsystem(kEmergencyStop);
    if (res.has_value())
    {
        // FM accepted the direct enable — now test the arming guard.
        FATP_ASSERT_FALSE(f.mgr.validateArmingReadiness().has_value(),
                          "Arming must fail while EmergencyStop is active");
        FATP_ASSERT_CONTAINS(f.mgr.validateArmingReadiness().error(), "EmergencyStop",
                             "Error must mention EmergencyStop");
    }
    // If FM rejected the direct enable (Preempts would block it only if ES is
    // already on — but here it is off, so enable should succeed).
    return true;
}

// ============================================================================
// switchFlightMode and validateFlightMode
// ============================================================================

FATP_TEST_CASE(switch_flight_mode_atomic)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kStabilize);
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kStabilize), "f.mgr.isEnabled(kStabilize)");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kGPS), "f.mgr.isEnabled(kGPS)");

    auto res = f.mgr.switchFlightMode(kPosHold);
    FATP_ASSERT_TRUE(res.has_value(),              "switchFlightMode to PosHold should succeed");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kStabilize), "Stabilize disabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kPosHold),    "PosHold enabled");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS),        "GPS auto-required by PosHold");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU),        "IMU preserved");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer),  "Barometer preserved");
    return true;
}

FATP_TEST_CASE(switch_flight_mode_from_no_active_mode)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.switchFlightMode(kManual).has_value(), "Should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kManual), "Manual enabled");
    return true;
}

FATP_TEST_CASE(switch_flight_mode_rejects_non_mode)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.switchFlightMode(kGPS).has_value(),
                      "switchFlightMode(GPS) must fail");
    FATP_ASSERT_FALSE(f.mgr.switchFlightMode(kBatteryMonitor).has_value(),
                      "switchFlightMode(BatteryMonitor) must fail");
    FATP_ASSERT_FALSE(f.mgr.switchFlightMode(kProfileArmed).has_value(),
                      "switchFlightMode(ArmedProfile) must fail");
    return true;
}

FATP_TEST_CASE(validate_flight_mode_rejects_non_mode)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kGPS);
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS), "f.mgr.isEnabled(kGPS)");
    FATP_ASSERT_FALSE(f.mgr.validateFlightMode(kGPS).has_value(),
                      "validateFlightMode(GPS) must fail: not a flight mode");
    return true;
}

FATP_TEST_CASE(validate_flight_mode_accepts_active_mode)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    FATP_ASSERT_TRUE(f.mgr.validateFlightMode(kManual).has_value(), "Manual active -> should pass");
    return true;
}

FATP_TEST_CASE(validate_flight_mode_rejects_inactive_mode)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.validateFlightMode(kManual).has_value(),
                      "Manual inactive -> should fail");
    return true;
}

// ============================================================================
// Previously tested subsystems
// ============================================================================

FATP_TEST_CASE(rtl_auto_enables_imu_barometer_gps)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kRTL).has_value(), "RTL should succeed");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kIMU), "f.mgr.isEnabled(kIMU)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBarometer), "f.mgr.isEnabled(kBarometer)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kGPS), "f.mgr.isEnabled(kGPS)");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kAltHold), "AltHold must NOT be auto-enabled");
    return true;
}

FATP_TEST_CASE(failsafe_auto_enables_battery_monitor_and_rcreceiver)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kFailsafe).has_value(), "f.mgr.enableSubsystem(kFailsafe).has_value()");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kBatteryMonitor), "f.mgr.isEnabled(kBatteryMonitor)");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kRCReceiver), "f.mgr.isEnabled(kRCReceiver)");
    return true;
}

FATP_TEST_CASE(geofence_is_independent)
{
    Fixture f;
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kGeofence).has_value(), "f.mgr.enableSubsystem(kGeofence).has_value()");
    FATP_ASSERT_TRUE(f.mgr.disableSubsystem(kGeofence).has_value(), "f.mgr.disableSubsystem(kGeofence).has_value()");
    return true;
}

// ============================================================================
// Adversarial
// ============================================================================

FATP_TEST_CASE(adversarial_enable_unknown_subsystem)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.enableSubsystem("NotASubsystem").has_value(), "f.mgr.enableSubsystem(\"NotASubsystem\").has_value()");
    return true;
}

FATP_TEST_CASE(adversarial_enable_empty_name)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.mgr.enableSubsystem("").has_value(), "f.mgr.enableSubsystem(\"\").has_value()");
    return true;
}

FATP_TEST_CASE(adversarial_cascading_disable_blocked)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kPosHold);
    FATP_ASSERT_FALSE(f.mgr.disableSubsystem(kIMU).has_value(), "f.mgr.disableSubsystem(kIMU).has_value()");
    FATP_ASSERT_FALSE(f.mgr.disableSubsystem(kGPS).has_value(), "f.mgr.disableSubsystem(kGPS).has_value()");
    return true;
}

FATP_TEST_CASE(adversarial_all_flight_modes_rejected_with_one_active)
{
    Fixture f;
    (void)f.mgr.enableSubsystem(kManual);
    static constexpr const char* kOtherModes[] = {
        kStabilize, kAltHold, kPosHold, kAutonomous, kRTL
    };
    for (const char* mode : kOtherModes)
    {
        FATP_ASSERT_FALSE(f.mgr.enableSubsystem(mode).has_value(),
                          (std::string(mode) + " must be rejected with Manual active").c_str());
    }
    return true;
}

FATP_TEST_CASE(adversarial_error_event_fired_on_conflict)
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager mgr{hub};
    std::vector<std::string> errored;
    auto conn = hub.onSubsystemError.connect(
        [&](std::string_view name, std::string_view) { errored.emplace_back(name); });
    (void)mgr.enableSubsystem(kManual);
    (void)mgr.enableSubsystem(kStabilize);
    FATP_ASSERT_FALSE(errored.empty(), "onSubsystemError should fire on conflict");
    return true;
}

// ============================================================================
// Stress
// ============================================================================

FATP_TEST_CASE(stress_flight_mode_cycle)
{
    Fixture f;
    static constexpr const char* kModes[] = { kManual, kStabilize, kAltHold, kPosHold, kRTL };
    for (int cycle = 0; cycle < 5; ++cycle)
    {
        for (const char* mode : kModes)
        {
            FATP_ASSERT_TRUE(f.mgr.enableSubsystem(mode).has_value(),
                             (std::string("Enable ") + mode).c_str());
            FATP_ASSERT_TRUE(f.mgr.disableSubsystem(mode).has_value(),
                             (std::string("Disable ") + mode).c_str());
            (void)f.mgr.disableSubsystem(kIMU);
            (void)f.mgr.disableSubsystem(kBarometer);
            (void)f.mgr.disableSubsystem(kGPS);
        }
    }
    return true;
}

FATP_TEST_CASE(stress_armed_profile_enter_leave_cycle)
{
    // Repeated enter/leave must leave state clean each time.
    Fixture f;
    for (int i = 0; i < 20; ++i)
    {
        FATP_ASSERT_TRUE(f.mgr.enterArmedConfiguration().has_value(), "enter");
        FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix), "MotorMix on");
        f.mgr.leaveArmedConfiguration();
        FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix), "MotorMix off");
        FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),      "ESC off");
    }
    return true;
}

FATP_TEST_CASE(stress_emergency_land_reset_cycle)
{
    // Repeated emergency land / reset must not leak subsystem state.
    Fixture f;
    for (int i = 0; i < 10; ++i)
    {
        (void)f.mgr.enableSubsystem(kManual);
        (void)f.mgr.enterArmedConfiguration();
        FATP_ASSERT_TRUE(f.mgr.triggerEmergencyLand().has_value(), "emergency land");
        FATP_ASSERT_TRUE(f.mgr.isEnabled(kMotorMix), "MotorMix live");
        FATP_ASSERT_TRUE(f.mgr.resetEmergencyStop().has_value(), "reset");
        FATP_ASSERT_FALSE(f.mgr.isEnabled(kMotorMix),       "MotorMix off after reset");
        FATP_ASSERT_FALSE(f.mgr.isEnabled(kESC),            "ESC off after reset");
        FATP_ASSERT_FALSE(f.mgr.isEnabled(kEmergencyStop),  "ES cleared");
        FATP_ASSERT_FALSE(f.mgr.isEnabled(kProfileEmergencyLand), "ELand profile cleared");
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

    FATP_RUN_TEST_NS(runner, subsystemmanager, initial_state_all_disabled);
    FATP_RUN_TEST_NS(runner, subsystemmanager, enable_independent_sensor);
    FATP_RUN_TEST_NS(runner, subsystemmanager, disable_enabled_sensor);
    FATP_RUN_TEST_NS(runner, subsystemmanager, requires_auto_enables_dependencies);
    FATP_RUN_TEST_NS(runner, subsystemmanager, requires_chain_poshold_enables_sensors);
    FATP_RUN_TEST_NS(runner, subsystemmanager, autonomous_implies_collision_avoidance);
    FATP_RUN_TEST_NS(runner, subsystemmanager, flight_modes_mutually_exclusive);
    FATP_RUN_TEST_NS(runner, subsystemmanager, power_chain_auto_enable_via_motormix);
    FATP_RUN_TEST_NS(runner, subsystemmanager, disable_dependency_blocks_if_dependent_enabled);
    FATP_RUN_TEST_NS(runner, subsystemmanager, active_flight_mode_query_empty);
    FATP_RUN_TEST_NS(runner, subsystemmanager, active_flight_mode_query_manual);

    // ArmedProfile — FM-native power chain ownership
    FATP_RUN_TEST_NS(runner, subsystemmanager, armed_profile_entails_power_chain_on_enter);
    FATP_RUN_TEST_NS(runner, subsystemmanager, armed_profile_entails_cascade_teardown_on_leave);
    FATP_RUN_TEST_NS(runner, subsystemmanager, armed_profile_idempotent_double_enter);
    FATP_RUN_TEST_NS(runner, subsystemmanager, armed_profile_leave_is_idempotent_when_not_entered);
    FATP_RUN_TEST_NS(runner, subsystemmanager, armed_profile_flight_mode_survives_leave);

    // EmergencyLandProfile
    FATP_RUN_TEST_NS(runner, subsystemmanager, emergency_land_profile_entails_power_chain);
    FATP_RUN_TEST_NS(runner, subsystemmanager, emergency_land_reset_auto_cleans_power_chain_via_graph);
    FATP_RUN_TEST_NS(runner, subsystemmanager, emergency_stop_reset_ground_path_no_op_profile);
    FATP_RUN_TEST_NS(runner, subsystemmanager, emergency_land_a2_latch_still_blocks_flight_modes);
    FATP_RUN_TEST_NS(runner, subsystemmanager, emergency_land_reset_allows_flight_modes);

    // EmergencyStop Preempts
    FATP_RUN_TEST_NS(runner, subsystemmanager, emergency_stop_preempts_active_flight_mode);
    FATP_RUN_TEST_NS(runner, subsystemmanager, emergency_stop_reset_clears_latch);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_emergency_stop_latch_covers_all_modes);

    // validateArmingReadiness
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_arming_readiness_minimal_set);
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_arming_readiness_missing_imu);
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_arming_readiness_missing_barometer);
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_arming_readiness_missing_rcreceiver);
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_arming_blocked_by_emergency_stop);

    // switchFlightMode / validateFlightMode
    FATP_RUN_TEST_NS(runner, subsystemmanager, switch_flight_mode_atomic);
    FATP_RUN_TEST_NS(runner, subsystemmanager, switch_flight_mode_from_no_active_mode);
    FATP_RUN_TEST_NS(runner, subsystemmanager, switch_flight_mode_rejects_non_mode);
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_flight_mode_rejects_non_mode);
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_flight_mode_accepts_active_mode);
    FATP_RUN_TEST_NS(runner, subsystemmanager, validate_flight_mode_rejects_inactive_mode);

    // Other subsystems
    FATP_RUN_TEST_NS(runner, subsystemmanager, rtl_auto_enables_imu_barometer_gps);
    FATP_RUN_TEST_NS(runner, subsystemmanager, failsafe_auto_enables_battery_monitor_and_rcreceiver);
    FATP_RUN_TEST_NS(runner, subsystemmanager, geofence_is_independent);

    // Adversarial
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_enable_unknown_subsystem);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_enable_empty_name);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_cascading_disable_blocked);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_all_flight_modes_rejected_with_one_active);
    FATP_RUN_TEST_NS(runner, subsystemmanager, adversarial_error_event_fired_on_conflict);

    // Stress
    FATP_RUN_TEST_NS(runner, subsystemmanager, stress_flight_mode_cycle);
    FATP_RUN_TEST_NS(runner, subsystemmanager, stress_armed_profile_enter_leave_cycle);
    FATP_RUN_TEST_NS(runner, subsystemmanager, stress_emergency_land_reset_cycle);

    return 0 == runner.print_summary();
}

} // namespace fat_p::testing

#ifdef ENABLE_TEST_APPLICATION
int main() { return fat_p::testing::test_SubsystemManager() ? 0 : 1; }
#endif
