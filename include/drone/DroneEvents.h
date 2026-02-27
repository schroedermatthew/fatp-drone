#pragma once

/*
FATP_META:
  meta_version: 1
  component: DroneEvents
  file_role: public_header
  path: include/drone/DroneEvents.h
  namespace: drone::events
  layer: Domain
  summary: Typed Signal-based event bus for drone subsystem and vehicle state change notifications.
  api_stability: in_work
  related:
    tests:
      - components/DroneCore/tests/test_DroneCore.cpp
  hygiene:
    pragma_once: true
    include_guard: false
    defines_total: 0
    defines_unprefixed: 0
    undefs_total: 0
    includes_windows_h: false
*/

/**
 * @file DroneEvents.h
 * @brief Typed event bus for drone subsystem and vehicle state notifications.
 *
 * @details
 * All inter-component communication happens through these signals. Components
 * connect slots to the DroneEventHub at construction time. A future GUI attaches
 * to the same hub without touching domain logic.
 *
 * Usage:
 * @code
 * drone::events::DroneEventHub hub;
 *
 * // Subscribe
 * auto conn = hub.onSubsystemChanged.connect(
 *     [](std::string_view name, bool enabled) {
 *         std::cout << name << (enabled ? " ON" : " OFF") << "\n";
 *     });
 *
 * // Emit (called from SubsystemManager internals)
 * hub.onSubsystemChanged.emit("GPS", true);
 * @endcode
 *
 * @note Thread-safety: Signal is not thread-safe by default.
 *       Emit only from the single control thread.
 */

#include "Signal.h"

#include <string_view>

namespace drone::events
{

/**
 * @brief Central event hub for all drone state change notifications.
 *
 * Owned by the top-level controller (VehicleController in main.cpp).
 * Passed by reference to domain components at construction.
 * All signals are public for direct subscription by observers.
 */
struct DroneEventHub
{
    /// Fired when a subsystem is enabled or disabled.
    /// Args: subsystem name, isEnabled
    fat_p::Signal<void(std::string_view, bool)> onSubsystemChanged;

    /// Fired when a subsystem enable/disable request is rejected.
    /// Args: subsystem name, reason string
    fat_p::Signal<void(std::string_view, std::string_view)> onSubsystemError;

    /// Fired when the vehicle state machine transitions.
    /// Args: from state name, to state name
    fat_p::Signal<void(std::string_view, std::string_view)> onVehicleStateChanged;

    /// Fired when a requested state transition is rejected.
    /// Args: requested state name, reason string
    fat_p::Signal<void(std::string_view, std::string_view)> onTransitionRejected;

    /// Fired for any safety-critical event (failsafe, emergency stop).
    /// Args: event name
    fat_p::Signal<void(std::string_view)> onSafetyAlert;
};

} // namespace drone::events
