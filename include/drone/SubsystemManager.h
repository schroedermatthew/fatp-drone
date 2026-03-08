#pragma once

/*
FATP_META:
  meta_version: 1
  component: SubsystemManager
  file_role: public_header
  path: include/drone/SubsystemManager.h
  namespace: drone
  layer: Domain
  summary: Drone subsystem feature manager - wraps FeatureManager with the drone dependency graph.
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
 * @file SubsystemManager.h
 * @brief Drone subsystem management built on fat_p::feature::FeatureManager.
 *
 * @details
 * Registers all drone subsystems as features with their dependency, implication,
 * conflict, mutual-exclusion, and preemption relationships. The FeatureManager
 * handles all constraint enforcement automatically.
 *
 * Dependency graph summary:
 *
 * Flight modes are MutuallyExclusive — they cannot chain via Requires
 * (AltHold cannot Require Stabilize since they conflict with each other).
 * Each mode independently declares its own sensor requirements.
 *
 * - Stabilize  Requires  IMU, Barometer
 * - AltHold    Requires  IMU, Barometer
 * - PosHold    Requires  IMU, Barometer, GPS
 * - Autonomous Requires  IMU, Barometer, GPS, Datalink, CollisionAvoidance
 * - Autonomous Implies   CollisionAvoidance  (auto-enable on enable)
 * - RTL        Requires  IMU, Barometer, GPS
 * - MotorMix   Requires  ESC
 * - ESC        Requires  BatteryMonitor
 * - Failsafe   Requires  BatteryMonitor, RCReceiver
 * - FlightModes group: MutuallyExclusive (Manual, Stabilize, AltHold, PosHold, Autonomous, RTL)
 *
 * EmergencyStop graph edges (FM is the source of truth):
 *   EmergencyStop Preempts Manual, Stabilize, AltHold, PosHold, Autonomous, RTL
 *
 *   This means:
 *   - Enabling EmergencyStop force-disables all flight modes via the FM Preempts cascade.
 *   - While EmergencyStop is enabled the FM itself rejects any flight mode re-enable.
 *   - Disabling EmergencyStop (via resetEmergencyStop()) lifts the latch; flight modes
 *     may be re-enabled explicitly afterwards.
 *
 *   Two trigger paths, both activate EmergencyStop via forceExclusive():
 *     triggerEmergencyStop() - kill path: blanks all features including motors.
 *       Safe when already on the ground (Armed state).
 *     triggerEmergencyLand() - land path: forceExclusive() then re-enables the power
 *       chain (BatteryMonitor, ESC, MotorMix) so motors stay live for controlled descent.
 *       Used when airborne (Flying or Landing states).
 *   Call resetEmergencyStop() to clear the latch. The VehicleStateMachine calls
 *   restorePreflightInvariant() via PreflightState::on_entry() after every transition
 *   to Preflight to enforce the canonical subsystem state.
 *
 * Canonical preflight invariant (enforced by restorePreflightInvariant()):
 *   - No active flight mode
 *   - EmergencyStop == false  (cleared by resetEmergencyStop before on_entry fires)
 *   - MotorMix      == false
 *   - ESC           == false
 *   Sensors (IMU, Barometer, GPS, etc.), BatteryMonitor, RCReceiver, Telemetry may
 *   remain on; they are safe to monitor in preflight.
 *
 * @see fat_p::feature::FeatureManager
 */

#include "DroneEvents.h"
#include "Expected.h"
#include "FeatureManager.h"
#include "Subsystems.h"

#include <optional>
#include <string>
#include <vector>

namespace drone
{

/**
 * @brief Manages drone subsystem state with dependency and conflict enforcement.
 *
 * Wraps fat_p::feature::FeatureManager<fat_p::SingleThreadedPolicy> with a
 * drone-specific feature graph registered at construction.
 *
 * @note Thread-safety: NOT thread-safe. Use from the single control thread.
 */
class SubsystemManager
{
public:
    using Manager = fat_p::feature::FeatureManager<fat_p::SingleThreadedPolicy>;

    /**
     * @brief Constructs the subsystem manager and registers all features.
     *
     * @param events Event hub to fire on subsystem state changes.
     * @throws std::runtime_error if feature graph construction fails (indicates a bug).
     */
    explicit SubsystemManager(drone::events::DroneEventHub& events)
        : mEvents(events)
    {
        registerSubsystems();
        registerRelationships();
        registerGroups();
        wireObserver();
    }

    /**
     * @brief Returns a const reference to the underlying FeatureManager.
     *
     * Allows DOT export, JSON serialization, and group state queries.
     */
    [[nodiscard]] const Manager& manager() const noexcept
    {
        return mManager;
    }

    /**
     * @brief Enables a subsystem, automatically resolving Requires/Implies dependencies.
     *
     * Rejects the request if the subsystem is a flight mode and EmergencyStop is
     * currently active. This check is belt-and-suspenders: the FeatureManager graph
     * also enforces this via the EmergencyStop Preempts edges added in
     * registerRelationships(). The wrapper check produces a more readable error.
     *
     * @param name Subsystem name constant (e.g., drone::subsystems::kAutonomous).
     * @return Expected<void> on success, or error string describing the conflict,
     *         missing dependency, or active EmergencyStop latch.
     *
     * @note Complexity: O(d * log n) where d = dependency depth.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> enableSubsystem(std::string_view name)
    {
        if (isFlightMode(name) && mManager.isEnabled(std::string(subsystems::kEmergencyStop)))
        {
            std::string msg =
                std::string("cannot enable '") + std::string(name) + "' while EmergencyStop is active";
            mEvents.onSubsystemError.emit(name, msg);
            return fat_p::unexpected(std::move(msg));
        }

        auto res = mManager.enable(std::string(name));
        if (!res)
        {
            mEvents.onSubsystemError.emit(name, res.error());
        }
        return res;
    }

    /**
     * @brief Disables a subsystem.
     *
     * Fails if another enabled subsystem Requires this one.
     *
     * @param name Subsystem name constant.
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> disableSubsystem(std::string_view name)
    {
        auto res = mManager.disable(std::string(name));
        if (!res)
        {
            mEvents.onSubsystemError.emit(name, res.error());
        }
        return res;
    }

    /**
     * @brief Atomically disables the current flight mode and enables a new one.
     *
     * Uses replace() when a flight mode is already active, which satisfies the
     * MutuallyExclusive constraint in a single lock and preserves shared sensors
     * (IMU, Barometer) that both modes Require. Falls back to enableSubsystem()
     * when no flight mode is currently active.
     *
     * @param newMode Name of the flight mode to switch to. Must be one of the six
     *                flight mode constants (kManual, kStabilize, kAltHold, kPosHold,
     *                kAutonomous, kRTL). Non-flight-mode names are rejected.
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> switchFlightMode(std::string_view newMode)
    {
        if (!isFlightMode(newMode))
        {
            std::string msg = std::string("'") + std::string(newMode) + "' is not a flight mode";
            mEvents.onSubsystemError.emit(newMode, msg);
            return fat_p::unexpected(std::move(msg));
        }

        const std::string current = activeFlightMode();
        if (current.empty())
        {
            return enableSubsystem(newMode);
        }

        auto res = mManager.replace(current, std::string(newMode));
        if (!res)
        {
            mEvents.onSubsystemError.emit(newMode, res.error());
        }
        return res;
    }

    /**
     * @brief Kill-path emergency stop. Blanks all subsystem state including motors.
     *
     * Uses forceExclusive(), which atomically clears all desiredStates and enables
     * EmergencyStop. The Preempts edges from EmergencyStop to all flight modes in the
     * FM graph then hold those modes off until resetEmergencyStop() is called.
     * All sensors, flight modes, and power chain subsystems are disabled as a side
     * effect. Use when the vehicle is on the ground (Armed state) where cutting motor
     * power is safe.
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> triggerEmergencyStop()
    {
        auto res = mManager.forceExclusive(std::string(subsystems::kEmergencyStop));
        if (!res)
        {
            mEvents.onSubsystemError.emit(subsystems::kEmergencyStop, res.error());
        }
        return res;
    }

    /**
     * @brief Land-path emergency stop. Disables flight modes but keeps motors live.
     *
     * Uses forceExclusive() to atomically clear all feature state and enable
     * EmergencyStop (same Preempts latch as the kill path), then immediately
     * re-enables the power chain (BatteryMonitor -> ESC -> MotorMix) so the drone
     * can execute a controlled descent. Use when airborne (Flying or Landing states).
     *
     * The Preempts latch in the FM graph still blocks any flight mode from being
     * re-enabled until resetEmergencyStop() is called.
     *
     * @return Expected<void> on success, or error from forceExclusive or power chain
     *         re-enable. On partial failure the EmergencyStop latch is still active;
     *         the error is surfaced via onSubsystemError.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> triggerEmergencyLand()
    {
        using namespace drone::subsystems;

        // Step 1: forceExclusive sets the Preempts latch and clears everything.
        auto res = mManager.forceExclusive(std::string(kEmergencyStop));
        if (!res)
        {
            mEvents.onSubsystemError.emit(kEmergencyStop, res.error());
            return res;
        }

        // Step 2: Re-enable the power chain so motors stay live for controlled descent.
        // Enable in dependency order: BatteryMonitor first, then ESC (Requires BatteryMonitor),
        // then MotorMix (Requires ESC). Failure here is unexpected but not fatal -- the
        // EmergencyStop latch is already set; surface the error and return it.
        for (const char* name : {kBatteryMonitor, kESC, kMotorMix})
        {
            auto pw = mManager.enable(std::string(name));
            if (!pw)
            {
                mEvents.onSubsystemError.emit(name, pw.error());
                return pw;
            }
        }
        return {};
    }

    /**
     * @brief Clears the EmergencyStop latch so flight modes can be re-enabled.
     *
     * Disables the EmergencyStop feature. The Preempts latch in the FM graph is
     * lifted; previously preempted flight modes are not auto-re-enabled. Must be
     * called as part of the reset sequence before the vehicle state machine
     * transitions back to Preflight.
     *
     * The VehicleStateMachine calls restorePreflightInvariant() via
     * PreflightState::on_entry() after this returns, which ensures the motor chain
     * (MotorMix, ESC) is powered down before the Preflight state is entered.
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> resetEmergencyStop()
    {
        auto res = mManager.disable(std::string(subsystems::kEmergencyStop));
        if (!res)
        {
            mEvents.onSubsystemError.emit(subsystems::kEmergencyStop, res.error());
        }
        return res;
    }

    /**
     * @brief Restores the canonical preflight subsystem configuration.
     *
     * Called by PreflightState::on_entry() on every transition into Preflight,
     * regardless of the path (disarm, disarm_after_landing, reset). Guarantees
     * that named vehicle states correspond to consistent subsystem configurations.
     *
     * Post-condition:
     *   - All six flight modes are disabled.
     *   - MotorMix is disabled.
     *   - ESC is disabled.
     *   - Sensors, BatteryMonitor, RCReceiver, Telemetry are left untouched
     *     (safe to remain on for preflight setup and monitoring).
     *
     * Features are disabled in consumer-first order to satisfy Requires constraints.
     * Errors from features that are already disabled are silently ignored.
     *
     * @note Complexity: O(n) where n is the number of features to disable.
     */
    void restorePreflightInvariant()
    {
        using namespace drone::subsystems;

        // Disable flight modes first -- they have no dependents in the power chain.
        for (const char* mode : {kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL})
        {
            (void)mManager.disable(mode);
        }

        // Disable power chain in consumer-first order.
        // MotorMix Requires ESC; disable MotorMix before ESC.
        (void)mManager.disable(kMotorMix);
        (void)mManager.disable(kESC);
        // BatteryMonitor intentionally left on -- safe for preflight monitoring.
    }

    /**
     * @brief Returns true if the named subsystem is currently enabled.
     */
    [[nodiscard]] bool isEnabled(std::string_view name) const
    {
        return mManager.isEnabled(std::string(name));
    }

    /**
     * @brief Returns a list of all currently enabled subsystem names.
     */
    [[nodiscard]] std::vector<std::string> enabledSubsystems() const
    {
        using namespace drone::subsystems;

        // All feature names in registration order
        static constexpr const char* kAllFeatures[] = {
            kIMU, kGPS, kBarometer, kCompass, kOpticalFlow, kLidar,
            kBatteryMonitor, kESC, kMotorMix,
            kRCReceiver, kTelemetry, kDatalink,
            kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL,
            kGeofence, kFailsafe, kCollisionAvoid, kEmergencyStop
        };

        std::vector<std::string> result;
        for (const char* name : kAllFeatures)
        {
            if (mManager.isEnabled(name))
            {
                result.emplace_back(name);
            }
        }
        return result;
    }

    /**
     * @brief Validates that the drone is ready to arm.
     *
     * Required for arming: IMU, Barometer, BatteryMonitor, ESC, MotorMix, RCReceiver.
     * Also rejects arming if EmergencyStop is active -- arming while the emergency
     * latch is set would bypass the safety interlock.
     *
     * @return Expected<void> on success, or error describing the first violation.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> validateArmingReadiness() const
    {
        using namespace drone::subsystems;

        // Reject arming with an active emergency latch first -- clearest failure mode.
        if (mManager.isEnabled(kEmergencyStop))
        {
            return fat_p::unexpected(
                std::string("Cannot arm: EmergencyStop is active. Call reset to clear."));
        }

        static constexpr const char* kArmRequired[] = {
            kIMU, kBarometer, kBatteryMonitor, kESC, kMotorMix, kRCReceiver
        };

        for (const char* name : kArmRequired)
        {
            if (!mManager.isEnabled(name))
            {
                return fat_p::unexpected(std::string("Arming requires '") + name + "' to be enabled");
            }
        }
        return {};
    }

    /**
     * @brief Validates that the given flight mode can be activated right now.
     *
     * Checks that the name is one of the six flight mode constants and that it is
     * currently enabled (dependency chain already satisfied by FeatureManager on
     * enable).
     *
     * @param mode Flight mode name (e.g., drone::subsystems::kPosHold).
     * @return Expected<void> on success, or error if mode is not a flight mode
     *         or is not currently active.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> validateFlightMode(std::string_view mode) const
    {
        if (!isFlightMode(mode))
        {
            return fat_p::unexpected(
                std::string("'") + std::string(mode) + "' is not a flight mode");
        }
        if (!mManager.isEnabled(std::string(mode)))
        {
            return fat_p::unexpected(
                std::string("Flight mode '") + std::string(mode) + "' is not active");
        }
        return {};
    }

    /**
     * @brief Returns any currently active flight mode name, or empty string if none.
     */
    [[nodiscard]] std::string activeFlightMode() const
    {
        using namespace drone::subsystems;

        static constexpr const char* kModes[] = {
            kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL
        };

        for (const char* mode : kModes)
        {
            if (mManager.isEnabled(mode))
            {
                return mode;
            }
        }
        return {};
    }

    /**
     * @brief Exports the subsystem dependency graph in GraphViz DOT format.
     */
    [[nodiscard]] std::string exportDependencyGraph() const
    {
        return mManager.toDot();
    }

    /**
     * @brief Serializes the current subsystem state to JSON.
     */
    [[nodiscard]] std::string toJson() const
    {
        return mManager.toJson();
    }

private:
    Manager mManager;
    drone::events::DroneEventHub& mEvents;
    // ScopedObserver has no default constructor; use optional so wireObserver()
    // can emplace it after mManager is fully constructed.
    std::optional<Manager::ScopedObserver> mObserver;

    // Throw on construction failure -- a bug in graph setup, not a runtime condition.
    static void requireOk(fat_p::Expected<void, std::string>&& res, const char* context)
    {
        if (!res)
        {
            throw std::runtime_error(std::string(context) + ": " + res.error());
        }
    }

    // Returns true if name is one of the six flight mode constants.
    static bool isFlightMode(std::string_view name)
    {
        using namespace drone::subsystems;

        static constexpr const char* kModes[] = {
            kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL
        };
        for (const char* mode : kModes)
        {
            if (name == mode) { return true; }
        }
        return false;
    }

    void registerSubsystems()
    {
        using namespace drone::subsystems;

        // Sensors
        for (const char* name : {kIMU, kGPS, kBarometer, kCompass, kOpticalFlow, kLidar})
        {
            requireOk(mManager.addFeature(name), "addFeature sensors");
        }

        // Power
        for (const char* name : {kBatteryMonitor, kESC, kMotorMix})
        {
            requireOk(mManager.addFeature(name), "addFeature power");
        }

        // Comms
        for (const char* name : {kRCReceiver, kTelemetry, kDatalink})
        {
            requireOk(mManager.addFeature(name), "addFeature comms");
        }

        // Flight modes
        for (const char* name : {kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL})
        {
            requireOk(mManager.addFeature(name), "addFeature flight modes");
        }

        // Safety
        for (const char* name : {kGeofence, kFailsafe, kCollisionAvoid, kEmergencyStop})
        {
            requireOk(mManager.addFeature(name), "addFeature safety");
        }
    }

    void registerRelationships()
    {
        using namespace drone::subsystems;
        using FR = fat_p::feature::FeatureRelationship;

        // Power chain
        requireOk(mManager.addRelationship(kESC,      FR::Requires, kBatteryMonitor), "ESC->BatteryMonitor");
        requireOk(mManager.addRelationship(kMotorMix, FR::Requires, kESC),            "MotorMix->ESC");

        // Safety
        requireOk(mManager.addRelationship(kFailsafe, FR::Requires, kBatteryMonitor), "Failsafe->BatteryMonitor");
        requireOk(mManager.addRelationship(kFailsafe, FR::Requires, kRCReceiver),     "Failsafe->RCReceiver");

        // Flight mode sensor requirements.
        // NOTE: Flight modes are MutuallyExclusive -- they cannot chain via Requires
        // (AltHold cannot Require Stabilize since they conflict with each other).
        // Each mode independently declares the sensors it needs.

        requireOk(mManager.addRelationship(kStabilize,  FR::Requires, kIMU),       "Stabilize->IMU");
        requireOk(mManager.addRelationship(kStabilize,  FR::Requires, kBarometer), "Stabilize->Barometer");

        requireOk(mManager.addRelationship(kAltHold,    FR::Requires, kIMU),       "AltHold->IMU");
        requireOk(mManager.addRelationship(kAltHold,    FR::Requires, kBarometer), "AltHold->Barometer");

        requireOk(mManager.addRelationship(kPosHold,    FR::Requires, kIMU),       "PosHold->IMU");
        requireOk(mManager.addRelationship(kPosHold,    FR::Requires, kBarometer), "PosHold->Barometer");
        requireOk(mManager.addRelationship(kPosHold,    FR::Requires, kGPS),       "PosHold->GPS");

        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kIMU),            "Auto->IMU");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kBarometer),      "Auto->Barometer");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kGPS),            "Auto->GPS");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kDatalink),       "Auto->Datalink");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kCollisionAvoid), "Auto->CollisionAvoid");
        requireOk(mManager.addRelationship(kAutonomous, FR::Implies,  kCollisionAvoid), "Auto=>CollisionAvoid");

        requireOk(mManager.addRelationship(kRTL,        FR::Requires, kIMU),       "RTL->IMU");
        requireOk(mManager.addRelationship(kRTL,        FR::Requires, kBarometer), "RTL->Barometer");
        requireOk(mManager.addRelationship(kRTL,        FR::Requires, kGPS),       "RTL->GPS");

        // EmergencyStop Preempts all flight modes.
        //
        // This is the FM-level source of truth for the emergency latch:
        //   - Enabling EmergencyStop force-disables all flight modes via Preempts cascade.
        //   - While EmergencyStop is enabled the FM rejects any flight mode re-enable
        //     (Preempts invariant: source enabled -> all targets must be disabled).
        //   - Disabling EmergencyStop lifts the latch; flight modes are not auto-re-enabled.
        //
        // The manual check in enableSubsystem() is belt-and-suspenders, providing a
        // more specific error message before the FM rejects via graph constraint.
        for (const char* mode : {kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL})
        {
            requireOk(mManager.addRelationship(kEmergencyStop, FR::Preempts, mode),
                      "EmergencyStop Preempts flight mode");
        }
    }

    void registerGroups()
    {
        using namespace drone::subsystems;

        requireOk(mManager.addGroup(kGroupSensors,
                      std::vector<std::string>{kIMU, kGPS, kBarometer, kCompass, kOpticalFlow, kLidar}),
                  "addGroup Sensors");

        requireOk(mManager.addGroup(kGroupPower,
                      std::vector<std::string>{kBatteryMonitor, kESC, kMotorMix}),
                  "addGroup Power");

        requireOk(mManager.addGroup(kGroupComms,
                      std::vector<std::string>{kRCReceiver, kTelemetry, kDatalink}),
                  "addGroup Comms");

        // MutuallyExclusive group: adds Conflicts between every pair of flight modes
        requireOk(mManager.addMutuallyExclusiveGroup(kGroupFlightModes,
                      std::vector<std::string>{kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL}),
                  "addMutuallyExclusiveGroup FlightModes");

        requireOk(mManager.addGroup(kGroupSafety,
                      std::vector<std::string>{kGeofence, kFailsafe, kCollisionAvoid, kEmergencyStop}),
                  "addGroup Safety");
    }

    void wireObserver()
    {
        mObserver.emplace(
            mManager,
            [this](const std::string& featureName, bool enabled, bool /*success*/)
            {
                mEvents.onSubsystemChanged.emit(featureName, enabled);
            });
    }
};

} // namespace drone
