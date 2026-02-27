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
    FATP_ASSERT_TRUE(res.has_value(), "Reset should succeed from Emergency");
    FATP_ASSERT_TRUE(f.sm.isPreflight(), "Should return to Preflight after reset");
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

namespace fat_p::testing
{

bool test_VehicleStateMachine()
{
    FATP_PRINT_HEADER(VEHICLE STATE MACHINE)

    TestRunner runner;

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

    return 0 == runner.print_summary();
}

} // namespace fat_p::testing

#ifdef ENABLE_TEST_APPLICATION
int main()
{
    return fat_p::testing::test_VehicleStateMachine() ? 0 : 1;
}
#endif
