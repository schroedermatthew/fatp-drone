#pragma once

/*
FATP_META:
  meta_version: 1
  component: DroneSubsystems
  file_role: public_header
  path: include/drone/Subsystems.h
  namespace: drone::subsystems
  layer: Domain
  summary: Subsystem name string constants and group name constants for the drone control system.
  api_stability: in_work
  related:
    tests:
      - components/SubsystemManager/tests/test_SubsystemManager.cpp
  hygiene:
    pragma_once: true
    include_guard: false
    defines_total: 0
    defines_unprefixed: 0
    undefs_total: 0
    includes_windows_h: false
*/

/**
 * @file Subsystems.h
 * @brief Subsystem name constants for the drone control system.
 *
 * All string literals used as FeatureManager keys live here.
 * Never scatter magic strings through the codebase.
 *
 * Internal profile features (kProfileArmed, kProfileEmergencyLand) are
 * registered in the FeatureManager graph but are not exposed as user commands.
 * They exist to encode subsystem configurations as graph-native relationships:
 *
 *   kProfileArmed       -- owned by the ArmedState; Entails the power chain
 *                          (MotorMix, ESC). Disabling it auto-tears-down motors
 *                          via FM ref-counted Entails semantics.
 *
 *   kProfileEmergencyLand -- owned by triggerEmergencyLand(); Entails the power
 *                          chain so motors stay live during controlled descent.
 *                          Disabling it (in resetEmergencyStop()) auto-cleans
 *                          the motor chain without imperative loops.
 */

namespace drone::subsystems
{

// ============================================================================
// Sensor subsystems
// ============================================================================

inline constexpr const char* kIMU          = "IMU";
inline constexpr const char* kGPS          = "GPS";
inline constexpr const char* kBarometer    = "Barometer";
inline constexpr const char* kCompass      = "Compass";
inline constexpr const char* kOpticalFlow  = "OpticalFlow";
inline constexpr const char* kLidar        = "Lidar";

// ============================================================================
// Power subsystems
// ============================================================================

inline constexpr const char* kBatteryMonitor = "BatteryMonitor";
inline constexpr const char* kESC            = "ESC";
inline constexpr const char* kMotorMix       = "MotorMix";

// ============================================================================
// Communications subsystems
// ============================================================================

inline constexpr const char* kRCReceiver = "RCReceiver";
inline constexpr const char* kTelemetry  = "Telemetry";
inline constexpr const char* kDatalink   = "Datalink";

// ============================================================================
// Flight modes (members of the MutuallyExclusive FlightModes group)
// ============================================================================

inline constexpr const char* kManual     = "Manual";
inline constexpr const char* kStabilize  = "Stabilize";
inline constexpr const char* kAltHold    = "AltHold";
inline constexpr const char* kPosHold    = "PosHold";
inline constexpr const char* kAutonomous = "Autonomous";
inline constexpr const char* kRTL        = "RTL"; ///< Return to Launch

// ============================================================================
// Safety subsystems
// ============================================================================

inline constexpr const char* kGeofence      = "Geofence";
inline constexpr const char* kFailsafe      = "Failsafe";
inline constexpr const char* kCollisionAvoid = "CollisionAvoidance";
inline constexpr const char* kEmergencyStop = "EmergencyStop";

// ============================================================================
// Internal profile features — managed by the state machine, not user commands
//
// These are registered in the FM graph so their Entails and Preempts
// relationships are enforced by the graph, not by imperative code.
// CommandParser blocks raw enable/disable of these names.
// ============================================================================

///< Enabled by ArmedState::on_entry; disabled by PreflightState::on_entry.
///< Entails MotorMix and ESC so the power chain is owned by the Armed state.
inline constexpr const char* kProfileArmed        = "ArmedProfile";

///< Enabled by triggerEmergencyLand(); disabled by resetEmergencyStop().
///< Entails MotorMix and ESC so motors stay live for controlled descent.
inline constexpr const char* kProfileEmergencyLand = "EmergencyLandProfile";

// ============================================================================
// Group names
// ============================================================================

inline constexpr const char* kGroupSensors     = "Sensors";
inline constexpr const char* kGroupPower       = "Power";
inline constexpr const char* kGroupComms       = "Comms";
inline constexpr const char* kGroupFlightModes = "FlightModes";
inline constexpr const char* kGroupSafety      = "Safety";

} // namespace drone::subsystems
