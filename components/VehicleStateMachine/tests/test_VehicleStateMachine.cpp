/**
 * @file test_VehicleStateMachine.cpp
 * @brief Unit tests for VehicleStateMachine.h
 *
 * Tests cover: initial state, valid transitions, guard rejections,
 * emergency reachable from flying/armed/landing, and reset.
 *
 * Arming fixture note: FeatureManager Requires cascade auto-enables sensor
 * dependencies, so we only need to explicitly enable leaf nodes (BatteryMonitor,
 * RCReceiver, Manual) plus IMU/Barometer for the arming check. The power chain
 * MotorMix->ESC->BatteryMonitor is activated by enabling MotorMix.
 */
/*
FATP_META:
  meta_version: 1
  component: VehicleStateMachine
  file_role: test
  path: components/VehicleStateMachine/tests/test_VehicleStateMachine.cpp
  namespace: fat_p::testing::vehiclestatemachine
  layer: Testing
  summary: Unit tests for VehicleStateMachine - guard logic and state transitions.
  api_stability: in_work
  related:
    headers:
      - include/drone/VehicleStateMachine.h
      - include/drone/SubsystemManager.h
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
#include "VehicleStateMachine.h"

namespace fat_p::testing::vehiclestatemachine
{

using namespace drone::subsystems;

struct Fixture
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager      mgr{hub};
    drone::VehicleStateMachine   sm{mgr, hub};

    /// Enable the six subsystems required by validateArmingReadiness:
    ///   IMU, Barometer, BatteryMonitor, ESC, MotorMix, RCReceiver.
    /// MotorMix Requires ESC Requires BatteryMonitor (auto-enabled),
    /// so enabling MotorMix is sufficient for the power chain.
    void enableArmingSubsystems()
    {
        (void)mgr.enableSubsystem(kIMU);
        (void)mgr.enableSubsystem(kBarometer);
        (void)mgr.enableSubsystem(kMotorMix);   // auto-enables ESC + BatteryMonitor
        (void)mgr.enableSubsystem(kRCReceiver);
    }

    /// Enable arming subsystems + Manual flight mode (no sensor deps beyond arming).
    void enableArmingAndManual()
    {
        enableArmingSubsystems();
        (void)mgr.enableSubsystem(kManual);
    }

    void goFlying()
    {
        enableArmingAndManual();
        (void)sm.requestArm();
        (void)sm.requestTakeoff();
    }

    void goLanding()
    {
        goFlying();
        (void)sm.requestLand();
    }
};

// ============================================================================
// Tests
// ============================================================================

FATP_TEST_CASE(initial_state_is_preflight)
{
    Fixture f;

    FATP_ASSERT_TRUE(f.sm.isPreflight(),  "Initial state should be Preflight");
    FATP_ASSERT_FALSE(f.sm.isArmed(),     "Should not be Armed initially");
    FATP_ASSERT_FALSE(f.sm.isFlying(),    "Should not be Flying initially");
    FATP_ASSERT_FALSE(f.sm.isLanding(),   "Should not be Landing initially");
    FATP_ASSERT_FALSE(f.sm.isEmergency(), "Should not be Emergency initially");
    FATP_ASSERT_EQ(std::string(f.sm.currentStateName()), std::string("Preflight"),
                   "currentStateName should return Preflight");
    return true;
}

FATP_TEST_CASE(arm_fails_without_required_subsystems)
{
    Fixture f;

    auto res = f.sm.requestArm();
    FATP_ASSERT_FALSE(res.has_value(), "Arming should fail without required subsystems");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain in Preflight after failed arm");
    return true;
}

FATP_TEST_CASE(arm_succeeds_with_required_subsystems)
{
    Fixture f;

    f.enableArmingSubsystems();
    auto res = f.sm.requestArm();
    FATP_ASSERT_TRUE(res.has_value(), "Arming should succeed with required subsystems");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should be in Armed state");
    FATP_ASSERT_EQ(std::string(f.sm.currentStateName()), std::string("Armed"),
                   "currentStateName should return Armed");
    return true;
}

FATP_TEST_CASE(disarm_from_armed)
{
    Fixture f;

    f.enableArmingSubsystems();
    (void)f.sm.requestArm();

    auto res = f.sm.requestDisarm();
    FATP_ASSERT_TRUE(res.has_value(), "Disarm should succeed from Armed");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should return to Preflight after disarm");
    return true;
}

FATP_TEST_CASE(disarm_fails_from_preflight)
{
    Fixture f;

    auto res = f.sm.requestDisarm();
    FATP_ASSERT_FALSE(res.has_value(), "Disarm should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain in Preflight");
    return true;
}

FATP_TEST_CASE(takeoff_fails_without_flight_mode)
{
    Fixture f;

    f.enableArmingSubsystems();
    (void)f.sm.requestArm();

    // No flight mode enabled
    auto res = f.sm.requestTakeoff();
    FATP_ASSERT_FALSE(res.has_value(), "Takeoff should fail without active flight mode");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

FATP_TEST_CASE(takeoff_succeeds_with_flight_mode)
{
    Fixture f;

    f.enableArmingAndManual();
    (void)f.sm.requestArm();

    auto res = f.sm.requestTakeoff();
    FATP_ASSERT_TRUE(res.has_value(), "Takeoff should succeed with Manual active");
    FATP_ASSERT_TRUE(f.sm.isFlying(), "Should be Flying");
    return true;
}

FATP_TEST_CASE(takeoff_fails_from_preflight)
{
    Fixture f;

    auto res = f.sm.requestTakeoff();
    FATP_ASSERT_FALSE(res.has_value(), "Takeoff from Preflight should fail");
    return true;
}

FATP_TEST_CASE(land_from_flying)
{
    Fixture f;

    f.enableArmingAndManual();
    (void)f.sm.requestArm();
    (void)f.sm.requestTakeoff();

    auto res = f.sm.requestLand();
    FATP_ASSERT_TRUE(res.has_value(), "Land should succeed from Flying");
    FATP_ASSERT_TRUE(f.sm.isLanding(), "Should be in Landing state");
    return true;
}

FATP_TEST_CASE(land_fails_from_armed)
{
    Fixture f;

    f.enableArmingSubsystems();
    (void)f.sm.requestArm();

    auto res = f.sm.requestLand();
    FATP_ASSERT_FALSE(res.has_value(), "Land should fail from Armed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

FATP_TEST_CASE(landing_complete_from_landing)
{
    Fixture f;

    f.enableArmingAndManual();
    (void)f.sm.requestArm();
    (void)f.sm.requestTakeoff();
    (void)f.sm.requestLand();

    auto res = f.sm.requestLandingComplete();
    FATP_ASSERT_TRUE(res.has_value(), "LandingComplete should succeed from Landing");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should return to Armed after landing complete");
    return true;
}

FATP_TEST_CASE(disarm_after_landing)
{
    Fixture f;

    f.enableArmingAndManual();
    (void)f.sm.requestArm();
    (void)f.sm.requestTakeoff();
    (void)f.sm.requestLand();

    auto res = f.sm.requestDisarmAfterLanding();
    FATP_ASSERT_TRUE(res.has_value(), "DisarmAfterLanding should succeed from Landing");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should return to Preflight");
    return true;
}

FATP_TEST_CASE(emergency_from_armed)
{
    Fixture f;

    f.enableArmingSubsystems();
    (void)f.sm.requestArm();

    auto res = f.sm.requestEmergency("test emergency from armed");
    FATP_ASSERT_TRUE(res.has_value(), "Emergency should succeed from Armed");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "Should be in Emergency state");
    return true;
}

FATP_TEST_CASE(emergency_from_flying)
{
    Fixture f;

    f.enableArmingAndManual();
    (void)f.sm.requestArm();
    (void)f.sm.requestTakeoff();

    auto res = f.sm.requestEmergency("test emergency from flying");
    FATP_ASSERT_TRUE(res.has_value(), "Emergency should succeed from Flying");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "Should be in Emergency state");
    return true;
}

FATP_TEST_CASE(emergency_from_landing)
{
    Fixture f;

    f.enableArmingAndManual();
    (void)f.sm.requestArm();
    (void)f.sm.requestTakeoff();
    (void)f.sm.requestLand();

    auto res = f.sm.requestEmergency("test emergency from landing");
    FATP_ASSERT_TRUE(res.has_value(), "Emergency should succeed from Landing");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "Should be in Emergency state");
    return true;
}

FATP_TEST_CASE(reset_from_emergency)
{
    Fixture f;

    f.enableArmingSubsystems();
    (void)f.sm.requestArm();
    (void)f.sm.requestEmergency("test");

    auto res = f.sm.requestReset();
    FATP_ASSERT_TRUE(res.has_value(),    "Reset should succeed from Emergency");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should return to Preflight after reset");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kEmergencyStop),
                      "EmergencyStop must be disabled after reset");
    return true;
}

FATP_TEST_CASE(reset_fails_from_preflight)
{
    Fixture f;

    auto res = f.sm.requestReset();
    FATP_ASSERT_FALSE(res.has_value(), "Reset should fail when not in Emergency");
    return true;
}

FATP_TEST_CASE(emergency_fails_from_preflight)
{
    Fixture f;

    auto res = f.sm.requestEmergency("should fail");
    FATP_ASSERT_FALSE(res.has_value(), "Emergency should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain in Preflight");
    return true;
}

FATP_TEST_CASE(state_transition_events_fired)
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager      mgr{hub};
    drone::VehicleStateMachine   sm{mgr, hub};

    std::vector<std::pair<std::string, std::string>> transitions;
    auto conn = hub.onVehicleStateChanged.connect(
        [&](std::string_view from, std::string_view to)
        {
            transitions.push_back({std::string(from), std::string(to)});
        });

    (void)mgr.enableSubsystem(kIMU);
    (void)mgr.enableSubsystem(kBarometer);
    (void)mgr.enableSubsystem(kMotorMix);
    (void)mgr.enableSubsystem(kRCReceiver);

    (void)sm.requestArm();

    bool foundArmedTransition = false;
    for (const auto& [from, to] : transitions)
    {
        if (to == "Armed") { foundArmedTransition = true; }
    }
    FATP_ASSERT_TRUE(foundArmedTransition, "Should have observed transition to Armed");
    return true;
}

FATP_TEST_CASE(transition_rejected_event_fired)
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager      mgr{hub};
    drone::VehicleStateMachine   sm{mgr, hub};

    std::string rejectedCmd;
    auto conn = hub.onTransitionRejected.connect(
        [&](std::string_view cmd, std::string_view)
        {
            rejectedCmd = std::string(cmd);
        });

    (void)sm.requestArm(); // fails without subsystems

    FATP_ASSERT_EQ(rejectedCmd, std::string("arm"), "Rejected command should be 'arm'");
    return true;
}

} // namespace fat_p::testing::vehiclestatemachine

// ============================================================================
// These tests are defined outside the inner namespace so the runner can find
// them — they are still in the vehiclestatemachine test suite.
// ============================================================================

namespace fat_p::testing::vehiclestatemachine
{

// ============================================================================
// Bug fix: full arm -> emergency -> reset -> re-arm -> takeoff cycle
// ============================================================================

FATP_TEST_CASE(reset_clears_emergency_stop_latch)
{
    // The full recovery path: arm, trigger emergency (kEmergencyStop latches),
    // reset (latch cleared), re-arm, takeoff. Any failure here is the Section 2 bug.
    Fixture f;
    f.enableArmingAndManual();

    FATP_ASSERT_TRUE(f.sm.requestArm().has_value(),             "arm");
    FATP_ASSERT_TRUE(f.sm.requestEmergency("engine").has_value(),"emergency");
    FATP_ASSERT_TRUE(f.mgr.isEnabled(kEmergencyStop),           "EmergencyStop latched after emergency");

    FATP_ASSERT_TRUE(f.sm.requestReset().has_value(),           "reset");
    FATP_ASSERT_TRUE(f.sm.isPreflight(),                        "back in Preflight");
    FATP_ASSERT_FALSE(f.mgr.isEnabled(kEmergencyStop),          "EmergencyStop cleared after reset");

    // Re-enable Manual (forceExclusive cleared it during emergency)
    FATP_ASSERT_TRUE(f.mgr.enableSubsystem(kManual).has_value(),"re-enable Manual after latch clear");
    FATP_ASSERT_TRUE(f.sm.requestArm().has_value(),             "re-arm after reset");
    FATP_ASSERT_TRUE(f.sm.requestTakeoff().has_value(),         "takeoff after re-arm");
    FATP_ASSERT_TRUE(f.sm.isFlying(),                           "flying after full recovery");
    return true;
}

// ============================================================================
// Adversarial — invalid transitions from every wrong state
// (merged from components/VehicleStateMachine/test_VehicleStateMachine.cpp)
// ============================================================================

FATP_TEST_CASE(adversarial_disarm_fails_from_flying)
{
    Fixture f;
    f.goFlying();
    FATP_ASSERT_FALSE(f.sm.requestDisarm().has_value(), "Disarm should fail from Flying");
    FATP_ASSERT_TRUE(f.sm.isFlying(), "Should remain Flying");
    return true;
}

FATP_TEST_CASE(adversarial_disarm_fails_from_landing)
{
    Fixture f;
    f.goLanding();
    FATP_ASSERT_FALSE(f.sm.requestDisarm().has_value(), "Disarm should fail from Landing");
    FATP_ASSERT_TRUE(f.sm.isLanding(), "Should remain Landing");
    return true;
}

FATP_TEST_CASE(adversarial_disarm_fails_from_emergency)
{
    Fixture f;
    f.enableArmingSubsystems();
    (void)f.sm.requestArm();
    (void)f.sm.requestEmergency("test");
    FATP_ASSERT_FALSE(f.sm.requestDisarm().has_value(), "Disarm should fail from Emergency");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "Should remain Emergency");
    return true;
}

FATP_TEST_CASE(adversarial_takeoff_fails_from_flying)
{
    Fixture f;
    f.goFlying();
    FATP_ASSERT_FALSE(f.sm.requestTakeoff().has_value(), "Takeoff should fail when already Flying");
    FATP_ASSERT_TRUE(f.sm.isFlying(), "Should remain Flying");
    return true;
}

FATP_TEST_CASE(adversarial_takeoff_fails_from_landing)
{
    Fixture f;
    f.goLanding();
    FATP_ASSERT_FALSE(f.sm.requestTakeoff().has_value(), "Takeoff should fail from Landing");
    FATP_ASSERT_TRUE(f.sm.isLanding(), "Should remain Landing");
    return true;
}

FATP_TEST_CASE(adversarial_land_fails_from_preflight)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.sm.requestLand().has_value(), "Land should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain Preflight");
    return true;
}

FATP_TEST_CASE(adversarial_land_fails_from_landing)
{
    Fixture f;
    f.goLanding();
    FATP_ASSERT_FALSE(f.sm.requestLand().has_value(), "Land should fail when already Landing");
    FATP_ASSERT_TRUE(f.sm.isLanding(), "Should remain Landing");
    return true;
}

FATP_TEST_CASE(adversarial_landing_complete_fails_from_armed)
{
    Fixture f;
    f.enableArmingSubsystems();
    (void)f.sm.requestArm();
    FATP_ASSERT_FALSE(f.sm.requestLandingComplete().has_value(),
                      "LandingComplete should fail from Armed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

FATP_TEST_CASE(adversarial_landing_complete_fails_from_preflight)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.sm.requestLandingComplete().has_value(),
                      "LandingComplete should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain Preflight");
    return true;
}

FATP_TEST_CASE(adversarial_disarm_after_landing_fails_from_armed)
{
    Fixture f;
    f.enableArmingSubsystems();
    (void)f.sm.requestArm();
    FATP_ASSERT_FALSE(f.sm.requestDisarmAfterLanding().has_value(),
                      "DisarmAfterLanding should fail from Armed (not Landing)");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

FATP_TEST_CASE(adversarial_disarm_after_landing_fails_from_preflight)
{
    Fixture f;
    FATP_ASSERT_FALSE(f.sm.requestDisarmAfterLanding().has_value(),
                      "DisarmAfterLanding should fail from Preflight");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should remain Preflight");
    return true;
}

FATP_TEST_CASE(adversarial_emergency_fails_from_emergency)
{
    Fixture f;
    f.enableArmingSubsystems();
    (void)f.sm.requestArm();
    (void)f.sm.requestEmergency("first");
    FATP_ASSERT_FALSE(f.sm.requestEmergency("second").has_value(),
                      "Emergency should fail when already in Emergency");
    FATP_ASSERT_TRUE(f.sm.isEmergency(), "Should remain in Emergency");
    return true;
}

FATP_TEST_CASE(adversarial_arm_fails_when_already_armed)
{
    Fixture f;
    f.enableArmingSubsystems();
    (void)f.sm.requestArm();
    FATP_ASSERT_FALSE(f.sm.requestArm().has_value(), "Arm should fail when already Armed");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

FATP_TEST_CASE(adversarial_reset_fails_from_armed)
{
    Fixture f;
    f.enableArmingSubsystems();
    (void)f.sm.requestArm();
    FATP_ASSERT_FALSE(f.sm.requestReset().has_value(), "Reset should fail from Armed (not Emergency)");
    FATP_ASSERT_TRUE(f.sm.isArmed(), "Should remain Armed");
    return true;
}

// ============================================================================
// Stress — repeated full flight cycles
// ============================================================================

FATP_TEST_CASE(stress_repeated_arm_disarm_cycles)
{
    Fixture f;
    f.enableArmingSubsystems();

    for (int i = 0; i < 20; ++i)
    {
        FATP_ASSERT_TRUE(f.sm.requestArm().has_value(),    "Arm should succeed on cycle");
        FATP_ASSERT_TRUE(f.sm.isArmed(),                   "Should be Armed");
        FATP_ASSERT_TRUE(f.sm.requestDisarm().has_value(), "Disarm should succeed on cycle");
        FATP_ASSERT_TRUE(f.sm.isPreflight(),               "Should return to Preflight");
    }
    return true;
}

FATP_TEST_CASE(stress_repeated_full_flight_cycle)
{
    for (int i = 0; i < 10; ++i)
    {
        drone::events::DroneEventHub hub;
        drone::SubsystemManager      mgr{hub};
        drone::VehicleStateMachine   sm{mgr, hub};

        (void)mgr.enableSubsystem(kIMU);
        (void)mgr.enableSubsystem(kBarometer);
        (void)mgr.enableSubsystem(kMotorMix);
        (void)mgr.enableSubsystem(kRCReceiver);
        (void)mgr.enableSubsystem(kManual);

        FATP_ASSERT_TRUE(sm.requestArm().has_value(),             "arm");
        FATP_ASSERT_TRUE(sm.requestTakeoff().has_value(),         "takeoff");
        FATP_ASSERT_TRUE(sm.requestLand().has_value(),            "land");
        FATP_ASSERT_TRUE(sm.requestLandingComplete().has_value(), "landing_complete");
        FATP_ASSERT_TRUE(sm.requestDisarm().has_value(),          "disarm");
        FATP_ASSERT_TRUE(sm.isPreflight(),                        "back to Preflight");
    }
    return true;
}

FATP_TEST_CASE(stress_emergency_and_reset_cycle)
{
    for (int i = 0; i < 15; ++i)
    {
        drone::events::DroneEventHub hub;
        drone::SubsystemManager      mgr{hub};
        drone::VehicleStateMachine   sm{mgr, hub};

        (void)mgr.enableSubsystem(kIMU);
        (void)mgr.enableSubsystem(kBarometer);
        (void)mgr.enableSubsystem(kMotorMix);
        (void)mgr.enableSubsystem(kRCReceiver);

        FATP_ASSERT_TRUE(sm.requestArm().has_value(),                    "arm for emergency cycle");
        FATP_ASSERT_TRUE(sm.requestEmergency("stress test").has_value(), "emergency");
        FATP_ASSERT_TRUE(sm.isEmergency(),                               "in Emergency");
        FATP_ASSERT_FALSE(mgr.isEnabled(kEmergencyStop) == false,        "EmergencyStop active");
        FATP_ASSERT_TRUE(sm.requestReset().has_value(),                  "reset");
        FATP_ASSERT_TRUE(sm.isPreflight(),                               "back to Preflight");
        FATP_ASSERT_FALSE(mgr.isEnabled(kEmergencyStop),                 "latch cleared after reset");
    }
    return true;
}

} // namespace fat_p::testing::vehiclestatemachine

namespace fat_p::testing
{

bool test_VehicleStateMachine()
{
    FATP_PRINT_HEADER(VEHICLE STATE MACHINE)

    TestRunner runner;

    // Basic / happy path
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, initial_state_is_preflight);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, arm_fails_without_required_subsystems);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, arm_succeeds_with_required_subsystems);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, disarm_from_armed);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, disarm_fails_from_preflight);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, takeoff_fails_without_flight_mode);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, takeoff_succeeds_with_flight_mode);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, takeoff_fails_from_preflight);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, land_from_flying);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, land_fails_from_armed);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, landing_complete_from_landing);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, disarm_after_landing);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, emergency_from_armed);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, emergency_from_flying);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, emergency_from_landing);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, reset_from_emergency);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, reset_fails_from_preflight);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, emergency_fails_from_preflight);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, state_transition_events_fired);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, transition_rejected_event_fired);

    // Bug fix: reset clears EmergencyStop latch (arm -> emergency -> reset -> re-arm -> takeoff)
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, reset_clears_emergency_stop_latch);

    // Adversarial — invalid transitions from every wrong state
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_disarm_fails_from_flying);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_disarm_fails_from_landing);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_disarm_fails_from_emergency);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_takeoff_fails_from_flying);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_takeoff_fails_from_landing);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_land_fails_from_preflight);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_land_fails_from_landing);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_landing_complete_fails_from_armed);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_landing_complete_fails_from_preflight);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_disarm_after_landing_fails_from_armed);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_disarm_after_landing_fails_from_preflight);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_emergency_fails_from_emergency);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_arm_fails_when_already_armed);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, adversarial_reset_fails_from_armed);

    // Stress
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, stress_repeated_arm_disarm_cycles);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, stress_repeated_full_flight_cycle);
    FATP_RUN_TEST_NS(runner, vehiclestatemachine, stress_emergency_and_reset_cycle);

    return 0 == runner.print_summary();
}

} // namespace fat_p::testing

#ifdef ENABLE_TEST_APPLICATION
int main()
{
    return fat_p::testing::test_VehicleStateMachine() ? 0 : 1;
}
#endif
