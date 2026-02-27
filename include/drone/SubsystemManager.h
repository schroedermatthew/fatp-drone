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
 * conflict, and mutual-exclusion relationships. The FeatureManager handles all
 * constraint enforcement automatically.
 *
 * Dependency graph summary:
 * - Stabilize  Requires  IMU, Barometer
 * - AltHold    Requires  Stabilize, Barometer
 * - PosHold    Requires  AltHold, GPS
 * - Autonomous Requires  PosHold, Datalink, CollisionAvoidance
 * - Autonomous Implies   CollisionAvoidance  (auto-enable)
 * - RTL        Requires  GPS, AltHold
 * - MotorMix   Requires  ESC
 * - ESC        Requires  BatteryMonitor
 * - Failsafe   Requires  BatteryMonitor, RCReceiver
 * - FlightModes group: MutuallyExclusive (Manual, Stabilize, AltHold, PosHold, Autonomous, RTL)
 * - EmergencyStop Conflicts all flight modes
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
     * @param name Subsystem name constant (e.g., drone::subsystems::kAutonomous).
     * @return Expected<void> on success, or error string describing the conflict or
     *         missing dependency.
     *
     * @note Complexity: O(d * log n) where d = dependency depth.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> enableSubsystem(std::string_view name)
    {
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
     *
     * @return Expected<void> on success, or error describing the first missing subsystem.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> validateArmingReadiness() const
    {
        using namespace drone::subsystems;

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
     * Checks that the mode is registered and currently enabled (dependency chain
     * already satisfied by FeatureManager on enable).
     *
     * @param mode Flight mode name (e.g., drone::subsystems::kPosHold).
     * @return Expected<void> on success, or error if mode is not active.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> validateFlightMode(std::string_view mode) const
    {
        if (!mManager.isEnabled(std::string(mode)))
        {
            return fat_p::unexpected(std::string("Flight mode '") + std::string(mode) + "' is not active");
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

    // Throw on construction failure — a bug in graph setup, not a runtime condition.
    static void requireOk(fat_p::Expected<void, std::string>&& res, const char* context)
    {
        if (!res)
        {
            throw std::runtime_error(std::string(context) + ": " + res.error());
        }
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
        // NOTE: Flight modes are MutuallyExclusive — they cannot chain via Requires
        // (AltHold cannot Require Stabilize since they conflict with each other).
        // Each mode independently declares the sensors it needs.

        // Stabilize: attitude control needs IMU + Barometer
        requireOk(mManager.addRelationship(kStabilize,  FR::Requires, kIMU),       "Stabilize->IMU");
        requireOk(mManager.addRelationship(kStabilize,  FR::Requires, kBarometer), "Stabilize->Barometer");

        // AltHold: altitude hold adds Barometer (IMU implied by any attitude-based mode)
        requireOk(mManager.addRelationship(kAltHold,    FR::Requires, kIMU),       "AltHold->IMU");
        requireOk(mManager.addRelationship(kAltHold,    FR::Requires, kBarometer), "AltHold->Barometer");

        // PosHold: position hold additionally needs GPS
        requireOk(mManager.addRelationship(kPosHold,    FR::Requires, kIMU),       "PosHold->IMU");
        requireOk(mManager.addRelationship(kPosHold,    FR::Requires, kBarometer), "PosHold->Barometer");
        requireOk(mManager.addRelationship(kPosHold,    FR::Requires, kGPS),       "PosHold->GPS");

        // Autonomous: full nav stack — GPS, Datalink, CollisionAvoidance
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kIMU),            "Auto->IMU");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kBarometer),      "Auto->Barometer");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kGPS),            "Auto->GPS");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kDatalink),       "Auto->Datalink");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kCollisionAvoid), "Auto->CollisionAvoid");
        // Enabling Autonomous auto-enables CollisionAvoidance via Implies cascade
        requireOk(mManager.addRelationship(kAutonomous, FR::Implies, kCollisionAvoid),  "Auto=>CollisionAvoid");

        // RTL: return-to-launch needs GPS + Barometer
        requireOk(mManager.addRelationship(kRTL,        FR::Requires, kIMU),       "RTL->IMU");
        requireOk(mManager.addRelationship(kRTL,        FR::Requires, kBarometer), "RTL->Barometer");
        requireOk(mManager.addRelationship(kRTL,        FR::Requires, kGPS),       "RTL->GPS");

        // EmergencyStop conflicts with all flight modes
        for (const char* mode : {kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL})
        {
            requireOk(mManager.addRelationship(kEmergencyStop, FR::Conflicts, mode), "EmergencyStop conflicts mode");
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
        // Emplace into optional: ScopedObserver has no default constructor.
        // Fires our lambda on every individual feature state change and
        // forwards it to DroneEventHub so TelemetryLog and the console can react.
        mObserver.emplace(
            mManager,
            [this](const std::string& featureName, bool enabled, bool /*success*/)
            {
                mEvents.onSubsystemChanged.emit(featureName, enabled);
            });
    }
};

} // namespace drone
