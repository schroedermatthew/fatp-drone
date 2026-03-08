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
 * The FeatureManager graph is the single source of truth for every subsystem
 * configuration. Named vehicle states correspond to specific FM graph states;
 * the state transitions that change subsystem configuration do so by enabling
 * or disabling profile features whose Entails and Preempts edges encode the
 * required configuration directly in the graph.
 *
 * -------------------------------------------------------------------------
 * Public feature graph
 * -------------------------------------------------------------------------
 *
 * Power chain (Requires):
 *   MotorMix  -> ESC -> BatteryMonitor
 *
 * Flight mode sensor requirements (Requires, each mode independent):
 *   Stabilize  -> IMU, Barometer
 *   AltHold    -> IMU, Barometer
 *   PosHold    -> IMU, Barometer, GPS
 *   Autonomous -> IMU, Barometer, GPS, Datalink, CollisionAvoidance
 *   Autonomous => CollisionAvoidance  (Implies: auto-enable)
 *   RTL        -> IMU, Barometer, GPS
 *
 * Safety (Requires):
 *   Failsafe -> BatteryMonitor, RCReceiver
 *
 * Flight mode exclusivity:
 *   FlightModes group: MutuallyExclusive (Manual, Stabilize, AltHold, PosHold, Autonomous, RTL)
 *
 * Emergency latch (Preempts):
 *   EmergencyStop Preempts every flight mode.
 *   While EmergencyStop is enabled the FM graph rejects any flight mode enable.
 *
 * -------------------------------------------------------------------------
 * Internal profile features (FM graph, not user-exposed)
 * -------------------------------------------------------------------------
 *
 * kProfileArmed ("ArmedProfile"):
 *   Entails MotorMix, ESC
 *   Enabled by ArmedState::on_entry (via enterArmedConfiguration()).
 *   Disabled by PreflightState::on_entry (via leaveArmedConfiguration()).
 *   Effect: when the vehicle enters Armed, FM-owned Entails bring up MotorMix
 *   and ESC. When the vehicle leaves Armed for Preflight, disabling the profile
 *   ref-count-decrements MotorMix and ESC -- if no other Entails source holds
 *   them, they go off automatically. No imperative cleanup loop needed.
 *
 * kProfileEmergencyLand ("EmergencyLandProfile"):
 *   Entails MotorMix, ESC
 *   Enabled by triggerEmergencyLand() after forceExclusive(EmergencyStop).
 *   Disabled by resetEmergencyStop() before the EmergencyStop latch is cleared.
 *   Effect: the Entails cascade re-owns the power chain so motors stay live
 *   for a controlled descent. Disabling it on reset auto-cleans the power chain.
 *
 * -------------------------------------------------------------------------
 * Arming readiness
 * -------------------------------------------------------------------------
 * validateArmingReadiness() requires: IMU, Barometer, RCReceiver + no active
 * EmergencyStop. Power chain (MotorMix/ESC/BatteryMonitor) is NOT required
 * from the user; it is brought up by ArmedProfile via Entails when the
 * Preflight->Armed transition fires.
 *
 * -------------------------------------------------------------------------
 * Preflight invariant (graph-derived, not imperative)
 * -------------------------------------------------------------------------
 * PreflightState::on_entry calls leaveArmedConfiguration(), which:
 *   1. Disables ArmedProfile  -> FM Entails ref-count takes down MotorMix, ESC
 *   2. Disables all flight modes
 * BatteryMonitor is NOT Entailed by ArmedProfile; it was auto-enabled by the
 * ESC Requires cascade and stays on for preflight monitoring.
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
     * @throws std::runtime_error if feature graph construction fails (bug).
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

    // -------------------------------------------------------------------------
    // User-facing subsystem enable/disable
    // -------------------------------------------------------------------------

    /**
     * @brief Enables a subsystem, automatically resolving Requires/Implies dependencies.
     *
     * Rejects the request if the subsystem is a flight mode and EmergencyStop is
     * currently active. This is belt-and-suspenders: the FM graph's Preempts edges
     * enforce the same rule. The wrapper produces a more readable error message.
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
     * MutuallyExclusive constraint in a single lock and preserves shared sensors.
     * Falls back to enableSubsystem() when no flight mode is currently active.
     *
     * @param newMode Must be one of the six flight mode constants. Non-flight-mode
     *                names are rejected.
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

    // -------------------------------------------------------------------------
    // State-machine hooks — called by VehicleStateMachine on_entry/on_exit
    // -------------------------------------------------------------------------

    /**
     * @brief Enables the ArmedProfile so the FM graph owns the power chain.
     *
     * Called by ArmedState::on_entry(). ArmedProfile Entails MotorMix and ESC;
     * enabling it triggers the Entails cascade which auto-enables MotorMix, then
     * the Requires cascade auto-enables ESC and BatteryMonitor.
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> enterArmedConfiguration()
    {
        auto res = mManager.enable(std::string(subsystems::kProfileArmed));
        if (!res)
        {
            mEvents.onSubsystemError.emit(subsystems::kProfileArmed, res.error());
        }
        return res;
    }

    /**
     * @brief Disables the ArmedProfile and all flight modes; restores Preflight state.
     *
     * Called by PreflightState::on_entry() on every path into Preflight
     * (disarm, disarm_after_landing, emergency reset).
     *
     * ArmedProfile owns MotorMix and ESC via Entails. Disabling it triggers
     * the FM ref-counted Entails cascade:
     *   - ArmedProfile disabled → MotorMix loses its Entails owner → off
     *   - planDisableClosure cascades through reverse deps → ESC disabled
     *     (MotorMix, which required ESC, is already in the plan as false)
     *   - BatteryMonitor: NOT Entailed; stays on for preflight monitoring
     *
     * Flight modes are disabled explicitly because they are user-managed
     * (not owned by any profile via Entails) and must be cleared to satisfy
     * the preflight invariant.
     *
     * @note All disable errors are silently ignored; features that are already
     *       off (e.g., after forceExclusive cleared them) produce no-op disables.
     * @note Complexity: O(n) where n is the number of features to disable.
     */
    void leaveArmedConfiguration()
    {
        using namespace drone::subsystems;

        // Disable ArmedProfile: FM Entails ref-count takes down MotorMix and ESC.
        (void)mManager.disable(kProfileArmed);

        // Flight modes are user-managed; disable them explicitly.
        for (const char* mode : {kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL})
        {
            (void)mManager.disable(mode);
        }
    }

    // -------------------------------------------------------------------------
    // Emergency paths
    // -------------------------------------------------------------------------

    /**
     * @brief Kill-path emergency stop. Blanks all subsystem state including motors.
     *
     * Uses forceExclusive(EmergencyStop), which atomically clears all desiredStates
     * and enables EmergencyStop. The FM Preempts edges block all flight modes until
     * resetEmergencyStop() is called. Use when on the ground (Armed state).
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
     * @brief Land-path emergency stop. Clears flight modes; re-owns power chain via FM graph.
     *
     * Two-step FM transaction:
     *   Step 1: forceExclusive(EmergencyStop) — atomically clears all feature state
     *           (including ArmedProfile and power chain) and sets the Preempts latch.
     *   Step 2: enable(EmergencyLandProfile) — ArmedProfile is gone; EmergencyLandProfile
     *           becomes the new Entails owner of MotorMix and ESC. The Entails cascade
     *           auto-enables MotorMix -> ESC -> BatteryMonitor so motors stay live.
     *
     * On reset, resetEmergencyStop() disables EmergencyLandProfile first. The FM
     * ref-counted Entails cascade takes down MotorMix and ESC automatically.
     * No cleanup loop. No imperative restore. The graph handles it.
     *
     * Use when airborne (Flying or Landing states).
     */
    [[nodiscard]] fat_p::Expected<void, std::string> triggerEmergencyLand()
    {
        using namespace drone::subsystems;

        // Step 1: forceExclusive sets the Preempts latch and blanks everything,
        // including any ArmedProfile and power chain features.
        auto res = mManager.forceExclusive(std::string(kEmergencyStop));
        if (!res)
        {
            mEvents.onSubsystemError.emit(kEmergencyStop, res.error());
            return res;
        }

        // Step 2: Enable EmergencyLandProfile. ArmedProfile is gone (cleared by
        // forceExclusive), so EmergencyLandProfile is now the sole Entails owner
        // of MotorMix and ESC. The Entails cascade re-enables the power chain.
        auto pw = mManager.enable(std::string(kProfileEmergencyLand));
        if (!pw)
        {
            mEvents.onSubsystemError.emit(kProfileEmergencyLand, pw.error());
            return pw;
        }
        return {};
    }

    /**
     * @brief Clears the emergency state and auto-cleans the power chain via FM graph.
     *
     * If EmergencyLandProfile is active (airborne path), disabling it triggers the
     * FM Entails ref-count cascade: MotorMix and ESC lose their sole Entails owner
     * and are disabled automatically. No explicit power-chain cleanup needed.
     *
     * Then EmergencyStop is disabled, lifting the FM Preempts latch on flight modes.
     *
     * For the ground-stop path (EmergencyLandProfile was never enabled),
     * the first disable is a no-op and only EmergencyStop is cleared.
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> resetEmergencyStop()
    {
        using namespace drone::subsystems;

        // Disable EmergencyLandProfile first (if active -- airborne path).
        // The FM Entails ref-count cascade auto-disables MotorMix and ESC.
        // Ignore error: on the ground path this profile was never enabled.
        (void)mManager.disable(kProfileEmergencyLand);

        // Now clear the EmergencyStop Preempts latch.
        auto res = mManager.disable(kEmergencyStop);
        if (!res)
        {
            mEvents.onSubsystemError.emit(kEmergencyStop, res.error());
        }
        return res;
    }

    // -------------------------------------------------------------------------
    // Queries
    // -------------------------------------------------------------------------

    /**
     * @brief Validates that the drone is ready to arm.
     *
     * Required: IMU, Barometer, RCReceiver.
     * The power chain (BatteryMonitor/ESC/MotorMix) is brought up by ArmedProfile
     * via Entails when the Preflight->Armed transition fires; it is NOT a user
     * prerequisite.
     *
     * Also rejects arming if EmergencyStop is active.
     *
     * @return Expected<void> on success, or error describing the first violation.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> validateArmingReadiness() const
    {
        using namespace drone::subsystems;

        if (mManager.isEnabled(kEmergencyStop))
        {
            return fat_p::unexpected(
                std::string("Cannot arm: EmergencyStop is active. Call reset to clear."));
        }

        static constexpr const char* kArmRequired[] = { kIMU, kBarometer, kRCReceiver };

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
     * Checks that the name is one of the six flight mode constants and that it
     * is currently enabled.
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
     * @brief Returns true if the named subsystem is currently enabled.
     */
    [[nodiscard]] bool isEnabled(std::string_view name) const
    {
        return mManager.isEnabled(std::string(name));
    }

    /**
     * @brief Returns the name of the currently active flight mode, or empty if none.
     */
    [[nodiscard]] std::string activeFlightMode() const
    {
        using namespace drone::subsystems;
        static constexpr const char* kModes[] = {
            kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL
        };
        for (const char* mode : kModes)
        {
            if (mManager.isEnabled(mode)) { return mode; }
        }
        return {};
    }

    /**
     * @brief Returns all currently enabled subsystem names.
     *
     * Includes internal profile features so the demo status output shows which
     * FM-owned profile is active (ArmedProfile, EmergencyLandProfile).
     */
    [[nodiscard]] std::vector<std::string> enabledSubsystems() const
    {
        using namespace drone::subsystems;

        static constexpr const char* kAllFeatures[] = {
            kIMU, kGPS, kBarometer, kCompass, kOpticalFlow, kLidar,
            kBatteryMonitor, kESC, kMotorMix,
            kRCReceiver, kTelemetry, kDatalink,
            kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL,
            kGeofence, kFailsafe, kCollisionAvoid, kEmergencyStop,
            kProfileArmed, kProfileEmergencyLand  // internal profiles
        };

        std::vector<std::string> result;
        for (const char* name : kAllFeatures)
        {
            if (mManager.isEnabled(name)) { result.emplace_back(name); }
        }
        return result;
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
    std::optional<Manager::ScopedObserver> mObserver;

    static void requireOk(fat_p::Expected<void, std::string>&& res, const char* context)
    {
        if (!res)
        {
            throw std::runtime_error(std::string(context) + ": " + res.error());
        }
    }

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

        for (const char* name : {kIMU, kGPS, kBarometer, kCompass, kOpticalFlow, kLidar})
            requireOk(mManager.addFeature(name), "addFeature sensors");

        for (const char* name : {kBatteryMonitor, kESC, kMotorMix})
            requireOk(mManager.addFeature(name), "addFeature power");

        for (const char* name : {kRCReceiver, kTelemetry, kDatalink})
            requireOk(mManager.addFeature(name), "addFeature comms");

        for (const char* name : {kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL})
            requireOk(mManager.addFeature(name), "addFeature flight modes");

        for (const char* name : {kGeofence, kFailsafe, kCollisionAvoid, kEmergencyStop})
            requireOk(mManager.addFeature(name), "addFeature safety");

        // Internal profile features.
        requireOk(mManager.addFeature(kProfileArmed),        "addFeature ArmedProfile");
        requireOk(mManager.addFeature(kProfileEmergencyLand),"addFeature EmergencyLandProfile");
    }

    void registerRelationships()
    {
        using namespace drone::subsystems;
        using FR = fat_p::feature::FeatureRelationship;

        // Power chain (Requires)
        requireOk(mManager.addRelationship(kESC,      FR::Requires, kBatteryMonitor), "ESC->BM");
        requireOk(mManager.addRelationship(kMotorMix, FR::Requires, kESC),            "MM->ESC");

        // Safety (Requires)
        requireOk(mManager.addRelationship(kFailsafe, FR::Requires, kBatteryMonitor), "Failsafe->BM");
        requireOk(mManager.addRelationship(kFailsafe, FR::Requires, kRCReceiver),     "Failsafe->RCR");

        // Flight mode sensor requirements
        requireOk(mManager.addRelationship(kStabilize,  FR::Requires, kIMU),       "Stab->IMU");
        requireOk(mManager.addRelationship(kStabilize,  FR::Requires, kBarometer), "Stab->Baro");
        requireOk(mManager.addRelationship(kAltHold,    FR::Requires, kIMU),       "AH->IMU");
        requireOk(mManager.addRelationship(kAltHold,    FR::Requires, kBarometer), "AH->Baro");
        requireOk(mManager.addRelationship(kPosHold,    FR::Requires, kIMU),       "PH->IMU");
        requireOk(mManager.addRelationship(kPosHold,    FR::Requires, kBarometer), "PH->Baro");
        requireOk(mManager.addRelationship(kPosHold,    FR::Requires, kGPS),       "PH->GPS");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kIMU),            "Au->IMU");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kBarometer),      "Au->Baro");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kGPS),            "Au->GPS");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kDatalink),       "Au->DL");
        requireOk(mManager.addRelationship(kAutonomous, FR::Requires, kCollisionAvoid), "Au->CA");
        requireOk(mManager.addRelationship(kAutonomous, FR::Implies,  kCollisionAvoid), "Au=>CA");
        requireOk(mManager.addRelationship(kRTL,        FR::Requires, kIMU),       "RTL->IMU");
        requireOk(mManager.addRelationship(kRTL,        FR::Requires, kBarometer), "RTL->Baro");
        requireOk(mManager.addRelationship(kRTL,        FR::Requires, kGPS),       "RTL->GPS");

        // EmergencyStop Preempts all flight modes.
        // FM source of truth for the emergency latch: while EmergencyStop is
        // enabled, the graph rejects any flight mode enable. The manual check
        // in enableSubsystem() is belt-and-suspenders for error message clarity.
        for (const char* mode : {kManual, kStabilize, kAltHold, kPosHold, kAutonomous, kRTL})
            requireOk(mManager.addRelationship(kEmergencyStop, FR::Preempts, mode),
                      "EmergencyStop Preempts mode");

        // ArmedProfile Entails the power chain (MotorMix, ESC).
        //
        // When ArmedProfile is enabled (ArmedState::on_entry):
        //   FM Entails cascade -> enable MotorMix -> Requires cascade -> ESC -> BatteryMonitor.
        //
        // When ArmedProfile is disabled (PreflightState::on_entry):
        //   FM Entails ref-count -> both MotorMix and ESC lose their sole Entails owner
        //   -> planDisableClosure processes MotorMix first (sets plan.desiredStates["MotorMix"]=false)
        //   -> then ESC: MotorMix is already false in the plan so its Requires edge
        //      does not block the ESC disable -> ESC disabled.
        //   BatteryMonitor: NOT Entailed; stays enabled for preflight monitoring.
        requireOk(mManager.addRelationship(kProfileArmed, FR::Entails, kMotorMix), "Armed Entails MM");
        requireOk(mManager.addRelationship(kProfileArmed, FR::Entails, kESC),      "Armed Entails ESC");

        // EmergencyLandProfile Entails the power chain (same edges as ArmedProfile).
        //
        // Enabled in triggerEmergencyLand() after forceExclusive(EmergencyStop):
        //   ArmedProfile was cleared by forceExclusive, so EmergencyLandProfile
        //   becomes the sole Entails owner. Entails cascade re-enables MotorMix/ESC.
        //
        // Disabled in resetEmergencyStop() (before EmergencyStop is cleared):
        //   FM Entails cascade auto-disables MotorMix and ESC. No cleanup loop.
        requireOk(mManager.addRelationship(kProfileEmergencyLand, FR::Entails, kMotorMix),
                  "ELand Entails MM");
        requireOk(mManager.addRelationship(kProfileEmergencyLand, FR::Entails, kESC),
                  "ELand Entails ESC");
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
