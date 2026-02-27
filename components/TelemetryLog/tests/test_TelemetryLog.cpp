/**
 * @file test_TelemetryLog.cpp
 * @brief Unit tests for TelemetryLog.h
 *
 * Tests cover: initial state, signal-driven entry appending,
 * capacity eviction, recent() query, formatTail(), and manual logInfo.
 */
/*
FATP_META:
  meta_version: 1
  component: TelemetryLog
  file_role: test
  path: components/TelemetryLog/tests/test_TelemetryLog.cpp
  namespace: fat_p::testing::telemetrylog
  layer: Testing
  summary: Unit tests for TelemetryLog - event capture and rolling log behavior.
  api_stability: in_work
  related:
    headers:
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

#include "DroneEvents.h"
#include "FatPTest.h"
#include "TelemetryLog.h"

namespace fat_p::testing::telemetrylog
{

FATP_TEST_CASE(initial_state_empty)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    FATP_ASSERT_TRUE(log.empty(), "Log should start empty");
    FATP_ASSERT_EQ(log.size(), std::size_t(0), "Log size should be 0");
    return true;
}

FATP_TEST_CASE(subsystem_enabled_event_recorded)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onSubsystemChanged.emit("GPS", true);

    FATP_ASSERT_EQ(log.size(), std::size_t(1), "One entry should be recorded");
    const auto& e = log.all().front();
    FATP_ASSERT_TRUE(e.category == drone::EventCategory::SubsystemEnabled,
                   "Category should be SubsystemEnabled");
    FATP_ASSERT_EQ(e.subject, std::string("GPS"), "Subject should be GPS");
    return true;
}

FATP_TEST_CASE(subsystem_disabled_event_recorded)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onSubsystemChanged.emit("IMU", false);

    FATP_ASSERT_TRUE(log.all().front().category ==
                   drone::EventCategory::SubsystemDisabled,
                   "Category should be SubsystemDisabled");
    return true;
}

FATP_TEST_CASE(subsystem_error_event_recorded)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onSubsystemError.emit("Stabilize", "IMU not enabled");

    const auto& e = log.all().front();
    FATP_ASSERT_TRUE(e.category == drone::EventCategory::SubsystemError, "Category should be SubsystemError");
    FATP_ASSERT_EQ(e.subject,  std::string("Stabilize"), "Subject should be Stabilize");
    FATP_ASSERT_CONTAINS(e.detail, "IMU", "Detail should mention IMU");
    return true;
}

FATP_TEST_CASE(state_transition_event_recorded)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onVehicleStateChanged.emit("Preflight", "Armed");

    const auto& e = log.all().front();
    FATP_ASSERT_TRUE(e.category == drone::EventCategory::StateTransition, "Category should be StateTransition");
    FATP_ASSERT_CONTAINS(e.detail, "Preflight", "Detail should mention from-state");
    FATP_ASSERT_CONTAINS(e.detail, "Armed",     "Detail should mention to-state");
    return true;
}

FATP_TEST_CASE(state_transition_initial_entry)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onVehicleStateChanged.emit("", "Preflight");

    FATP_ASSERT_CONTAINS(log.all().front().detail, "initial",
                         "Detail should say 'initial' for empty from-state");
    return true;
}

FATP_TEST_CASE(transition_rejected_event_recorded)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onTransitionRejected.emit("arm", "subsystems not ready");

    const auto& e = log.all().front();
    FATP_ASSERT_TRUE(e.category == drone::EventCategory::TransitionRejected, "Category should be TransitionRejected");
    FATP_ASSERT_EQ(e.subject, std::string("arm"), "Subject should be 'arm'");
    return true;
}

FATP_TEST_CASE(safety_alert_event_recorded)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onSafetyAlert.emit("battery critical");

    const auto& e = log.all().front();
    FATP_ASSERT_TRUE(e.category == drone::EventCategory::SafetyAlert, "Category should be SafetyAlert");
    FATP_ASSERT_EQ(e.subject, std::string("battery critical"), "Subject should match alert");
    return true;
}

FATP_TEST_CASE(multiple_events_accumulate)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onSubsystemChanged.emit("IMU",       true);
    hub.onSubsystemChanged.emit("GPS",       true);
    hub.onSubsystemChanged.emit("Barometer", true);
    hub.onVehicleStateChanged.emit("Preflight", "Armed");

    FATP_ASSERT_EQ(log.size(), std::size_t(4), "Four entries should be recorded");
    return true;
}

FATP_TEST_CASE(capacity_evicts_oldest)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<4>       log{hub};   // capacity = 4

    for (int i = 0; i < 6; ++i)
    {
        hub.onSubsystemChanged.emit("Sub" + std::to_string(i), true);
    }

    FATP_ASSERT_EQ(log.size(), std::size_t(4), "Log should be capped at MaxEntries=4");

    const auto entries = log.all();
    FATP_ASSERT_EQ(entries.front().subject, std::string("Sub2"),
                   "Oldest retained entry should be Sub2");
    FATP_ASSERT_EQ(entries.back().subject, std::string("Sub5"),
                   "Newest retained entry should be Sub5");
    return true;
}

FATP_TEST_CASE(recent_returns_tail)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    for (int i = 0; i < 10; ++i)
    {
        hub.onSubsystemChanged.emit("Sub" + std::to_string(i), true);
    }

    const auto tail = log.recent(3);
    FATP_ASSERT_EQ(tail.size(), std::size_t(3), "recent(3) should return 3 entries");
    FATP_ASSERT_EQ(tail[0].subject, std::string("Sub7"), "First of tail should be Sub7");
    FATP_ASSERT_EQ(tail[2].subject, std::string("Sub9"), "Last of tail should be Sub9");
    return true;
}

FATP_TEST_CASE(recent_clamped_when_n_exceeds_size)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onSubsystemChanged.emit("A", true);
    hub.onSubsystemChanged.emit("B", true);

    const auto tail = log.recent(100);
    FATP_ASSERT_EQ(tail.size(), std::size_t(2), "recent(100) should return all 2 entries");
    return true;
}

FATP_TEST_CASE(format_tail_produces_output)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onSubsystemChanged.emit("IMU", true);
    hub.onVehicleStateChanged.emit("Preflight", "Armed");

    const std::string output = log.formatTail(10);
    FATP_ASSERT_FALSE(output.empty(), "formatTail should produce non-empty output");
    FATP_ASSERT_CONTAINS(output, "ENABLED", "Output should contain ENABLED label");
    FATP_ASSERT_CONTAINS(output, "IMU",     "Output should contain IMU");
    FATP_ASSERT_CONTAINS(output, "STATE",   "Output should contain STATE label");
    return true;
}

FATP_TEST_CASE(format_tail_empty_log)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    const std::string output = log.formatTail(10);
    FATP_ASSERT_CONTAINS(output, "no telemetry", "Empty log should say 'no telemetry'");
    return true;
}

FATP_TEST_CASE(log_info_manual_entry)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    log.logInfo("session", "started");

    FATP_ASSERT_EQ(log.size(), std::size_t(1), "Manual logInfo should add one entry");
    const auto& e = log.all().front();
    FATP_ASSERT_TRUE(e.category == drone::EventCategory::Info,       "Category should be Info");
    FATP_ASSERT_EQ(e.subject,  std::string("session"),           "Subject should match");
    FATP_ASSERT_EQ(e.detail,   std::string("started"),           "Detail should match");
    return true;
}

FATP_TEST_CASE(clear_empties_log)
{
    drone::events::DroneEventHub hub;
    drone::TelemetryLog<64>      log{hub};

    hub.onSubsystemChanged.emit("IMU", true);
    hub.onSubsystemChanged.emit("GPS", true);

    log.clear();
    FATP_ASSERT_TRUE(log.empty(), "Log should be empty after clear");
    FATP_ASSERT_EQ(log.size(), std::size_t(0), "Size should be 0 after clear");
    return true;
}

FATP_TEST_CASE(connections_disconnect_on_destruction)
{
    drone::events::DroneEventHub hub;
    std::size_t captured = 0;

    {
        drone::TelemetryLog<64> log{hub};
        hub.onSubsystemChanged.emit("IMU", true);
        captured = log.size();
    }
    // log destroyed; ScopedConnections disconnected automatically

    // Post-destruction emit must not crash
    hub.onSubsystemChanged.emit("GPS", true);

    FATP_ASSERT_EQ(captured, std::size_t(1), "Should have captured one event before destruction");
    return true;
}

} // namespace fat_p::testing::telemetrylog

// ============================================================================
// Public interface
// ============================================================================

namespace fat_p::testing
{

bool test_TelemetryLog()
{
    FATP_PRINT_HEADER(TELEMETRY LOG)

    TestRunner runner;

    FATP_RUN_TEST_NS(runner, telemetrylog, initial_state_empty);
    FATP_RUN_TEST_NS(runner, telemetrylog, subsystem_enabled_event_recorded);
    FATP_RUN_TEST_NS(runner, telemetrylog, subsystem_disabled_event_recorded);
    FATP_RUN_TEST_NS(runner, telemetrylog, subsystem_error_event_recorded);
    FATP_RUN_TEST_NS(runner, telemetrylog, state_transition_event_recorded);
    FATP_RUN_TEST_NS(runner, telemetrylog, state_transition_initial_entry);
    FATP_RUN_TEST_NS(runner, telemetrylog, transition_rejected_event_recorded);
    FATP_RUN_TEST_NS(runner, telemetrylog, safety_alert_event_recorded);
    FATP_RUN_TEST_NS(runner, telemetrylog, multiple_events_accumulate);
    FATP_RUN_TEST_NS(runner, telemetrylog, capacity_evicts_oldest);
    FATP_RUN_TEST_NS(runner, telemetrylog, recent_returns_tail);
    FATP_RUN_TEST_NS(runner, telemetrylog, recent_clamped_when_n_exceeds_size);
    FATP_RUN_TEST_NS(runner, telemetrylog, format_tail_produces_output);
    FATP_RUN_TEST_NS(runner, telemetrylog, format_tail_empty_log);
    FATP_RUN_TEST_NS(runner, telemetrylog, log_info_manual_entry);
    FATP_RUN_TEST_NS(runner, telemetrylog, clear_empties_log);
    FATP_RUN_TEST_NS(runner, telemetrylog, connections_disconnect_on_destruction);

    return 0 == runner.print_summary();
}

} // namespace fat_p::testing

#ifdef ENABLE_TEST_APPLICATION
int main()
{
    return fat_p::testing::test_TelemetryLog() ? 0 : 1;
}
#endif
