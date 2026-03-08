#pragma once

/*
FATP_META:
  meta_version: 1
  component: VehicleStateMachine
  file_role: public_header
  path: include/drone/VehicleStateMachine.h
  namespace: drone
  layer: Domain
  summary: Drone vehicle lifecycle state machine with guard-protected transitions.
  api_stability: in_work
*/

/**
 * @file VehicleStateMachine.h
 * @brief Drone vehicle lifecycle state machine.
 *
 * @details
 * The VehicleStateMachine is responsible for lifecycle transitions only.
 * Subsystem configuration is the FM graph's job, driven by two hooks:
 *
 *   ArmedState::on_entry  -> subsystems.enterArmedConfiguration()
 *     Enables ArmedProfile; FM Entails cascade brings up MotorMix/ESC/BatteryMonitor.
 *
 *   PreflightState::on_entry -> subsystems.leaveArmedConfiguration()
 *     Disables ArmedProfile; FM Entails ref-count takes down MotorMix/ESC.
 *     Disables all flight modes (user-managed, not FM-owned).
 *
 * Every path into Preflight (disarm, disarm_after_landing, emergency reset) calls
 * on_entry exactly once. No state-specific cleanup is needed in on_exit hooks.
 *
 * Transition table:
 *   Preflight -> Armed       (arm; guard: validateArmingReadiness)
 *   Armed     -> Preflight   (disarm)
 *   Armed     -> Flying      (takeoff; guard: active flight mode)
 *   Flying    -> Landing     (land)
 *   Landing   -> Armed       (landing_complete)
 *   Landing   -> Preflight   (disarm_after_landing)
 *   Armed/Flying/Landing -> Emergency  (emergency; always)
 *   Emergency -> Preflight   (reset)
 */

#include "DroneEvents.h"
#include "Expected.h"
#include "StateMachine.h"
#include "SubsystemManager.h"

#include <array>
#include <string>

namespace drone
{

struct VehicleContext;

// ============================================================================
// States
// ============================================================================

struct PreflightState
{
    static constexpr const char* kName = "Preflight";
    void on_entry(VehicleContext& ctx);
    void on_exit(VehicleContext& ctx);
};

struct ArmedState
{
    static constexpr const char* kName = "Armed";
    void on_entry(VehicleContext& ctx);
    void on_exit(VehicleContext& ctx);
};

struct FlyingState
{
    static constexpr const char* kName = "Flying";
    void on_entry(VehicleContext& ctx);
    void on_exit(VehicleContext& ctx);
};

struct LandingState
{
    static constexpr const char* kName = "Landing";
    void on_entry(VehicleContext& ctx);
    void on_exit(VehicleContext& ctx);
};

struct EmergencyState
{
    static constexpr const char* kName = "Emergency";
    void on_entry(VehicleContext& ctx);
    void on_exit(VehicleContext& ctx);
};

// ============================================================================
// Transition table
// ============================================================================

using DroneTransitions = std::tuple<
    std::pair<PreflightState, ArmedState>,
    std::pair<ArmedState,     PreflightState>,
    std::pair<ArmedState,     FlyingState>,
    std::pair<FlyingState,    LandingState>,
    std::pair<LandingState,   ArmedState>,
    std::pair<LandingState,   PreflightState>,
    std::pair<ArmedState,     EmergencyState>,
    std::pair<FlyingState,    EmergencyState>,
    std::pair<LandingState,   EmergencyState>,
    std::pair<EmergencyState, PreflightState>
>;

// ============================================================================
// Context
// ============================================================================

struct VehicleContext
{
    SubsystemManager&             subsystems;
    drone::events::DroneEventHub& events;
    std::string lastError;   ///< Set by guard failures; cleared on successful transitions.
    std::string fromState;   ///< Name of the state being exited (set in on_exit).
};

// ============================================================================
// State on_entry / on_exit
// ============================================================================

inline void PreflightState::on_entry(VehicleContext& ctx)
{
    // leaveArmedConfiguration() is the single cleanup point for every path into
    // Preflight. It disables ArmedProfile (FM Entails cascade takes down
    // MotorMix/ESC) and clears all flight modes. No-ops if already clean.
    ctx.subsystems.leaveArmedConfiguration();
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}
inline void PreflightState::on_exit(VehicleContext& ctx)  { ctx.fromState = kName; }

inline void ArmedState::on_entry(VehicleContext& ctx)
{
    // enterArmedConfiguration enables ArmedProfile; the FM Entails cascade
    // brings up MotorMix -> ESC -> BatteryMonitor automatically.
    (void)ctx.subsystems.enterArmedConfiguration();
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}
inline void ArmedState::on_exit(VehicleContext& ctx)      { ctx.fromState = kName; }

inline void FlyingState::on_entry(VehicleContext& ctx)
{
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}
inline void FlyingState::on_exit(VehicleContext& ctx)     { ctx.fromState = kName; }

inline void LandingState::on_entry(VehicleContext& ctx)
{
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}
inline void LandingState::on_exit(VehicleContext& ctx)    { ctx.fromState = kName; }

inline void EmergencyState::on_entry(VehicleContext& ctx)
{
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}
inline void EmergencyState::on_exit(VehicleContext& ctx)  { ctx.fromState = kName; }

// ============================================================================
// State machine type alias
// ============================================================================

using DroneStateMachine = fat_p::StateMachine<
    VehicleContext,
    DroneTransitions,
    fat_p::StrictTransitionPolicy,
    fat_p::ThrowingActionPolicy,
    0,
    PreflightState,
    ArmedState,
    FlyingState,
    LandingState,
    EmergencyState
>;

// ============================================================================
// VehicleStateMachine — guard-protected wrapper
// ============================================================================

class VehicleStateMachine
{
public:
    VehicleStateMachine(SubsystemManager& subsystems, drone::events::DroneEventHub& events)
        : mContext{subsystems, events, {}, {}}
        , mSM(mContext)
    {}

    VehicleStateMachine(const VehicleStateMachine&) = delete;
    VehicleStateMachine& operator=(const VehicleStateMachine&) = delete;
    VehicleStateMachine(VehicleStateMachine&&) = delete;
    VehicleStateMachine& operator=(VehicleStateMachine&&) = delete;

    // -------------------------------------------------------------------------
    // State queries
    // -------------------------------------------------------------------------

    [[nodiscard]] std::string_view currentStateName() const noexcept
    {
        static constexpr std::array<const char*, 5> kNames = {
            PreflightState::kName, ArmedState::kName, FlyingState::kName,
            LandingState::kName,   EmergencyState::kName
        };
        return kNames[mSM.currentStateIndex()];
    }

    [[nodiscard]] bool isPreflight() const noexcept { return mSM.isInState<PreflightState>(); }
    [[nodiscard]] bool isArmed()     const noexcept { return mSM.isInState<ArmedState>(); }
    [[nodiscard]] bool isFlying()    const noexcept { return mSM.isInState<FlyingState>(); }
    [[nodiscard]] bool isLanding()   const noexcept { return mSM.isInState<LandingState>(); }
    [[nodiscard]] bool isEmergency() const noexcept { return mSM.isInState<EmergencyState>(); }

    // -------------------------------------------------------------------------
    // Transitions
    // -------------------------------------------------------------------------

    /**
     * @brief Preflight -> Armed.
     * Guard: validateArmingReadiness (IMU, Barometer, RCReceiver, no EmergencyStop).
     * on_entry: enterArmedConfiguration() brings up MotorMix/ESC/BatteryMonitor via graph.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestArm()
    {
        if (!isPreflight()) return reject("arm", "must be in Preflight state");
        auto guard = mContext.subsystems.validateArmingReadiness();
        if (!guard)       return reject("arm", guard.error());
        mSM.transition<ArmedState>();
        mContext.lastError.clear();
        return {};
    }

    /**
     * @brief Armed -> Preflight.
     * on_entry: leaveArmedConfiguration() tears down power chain via graph.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestDisarm()
    {
        if (!isArmed()) return reject("disarm", "must be in Armed state");
        mSM.transition<PreflightState>();
        mContext.lastError.clear();
        return {};
    }

    /**
     * @brief Armed -> Flying.
     * Guard: at least one flight mode must be active.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestTakeoff()
    {
        if (!isArmed()) return reject("takeoff", "must be in Armed state");
        if (mContext.subsystems.activeFlightMode().empty())
            return reject("takeoff",
                "no flight mode is active — enable Manual, Stabilize, AltHold, PosHold, Autonomous, or RTL");
        mSM.transition<FlyingState>();
        mContext.lastError.clear();
        return {};
    }

    /** @brief Flying -> Landing. */
    [[nodiscard]] fat_p::Expected<void, std::string> requestLand()
    {
        if (!isFlying()) return reject("land", "must be in Flying state");
        mSM.transition<LandingState>();
        mContext.lastError.clear();
        return {};
    }

    /** @brief Landing -> Armed. */
    [[nodiscard]] fat_p::Expected<void, std::string> requestLandingComplete()
    {
        if (!isLanding()) return reject("landing_complete", "must be in Landing state");
        mSM.transition<ArmedState>();
        mContext.lastError.clear();
        return {};
    }

    /**
     * @brief Landing -> Preflight.
     * on_entry: leaveArmedConfiguration() tears down power chain via graph.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestDisarmAfterLanding()
    {
        if (!isLanding()) return reject("disarm_after_landing", "must be in Landing state");
        mSM.transition<PreflightState>();
        mContext.lastError.clear();
        return {};
    }

    /**
     * @brief Triggers emergency (Armed/Flying/Landing -> Emergency).
     *
     * Ground path (Armed):   triggerEmergencyStop() — forceExclusive kills everything.
     * Airborne path (Flying/Landing): triggerEmergencyLand() — forceExclusive + enable
     *   EmergencyLandProfile; FM Entails cascade re-owns power chain for controlled descent.
     *
     * The SM transition proceeds regardless of subsystem call result; safety state
     * takes precedence. Errors surface via onSubsystemError.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestEmergency(std::string_view reason)
    {
        if (isEmergency() || isPreflight())
            return reject("emergency", "already in terminal state");

        if (isFlying() || isLanding())
            (void)mContext.subsystems.triggerEmergencyLand();
        else
            (void)mContext.subsystems.triggerEmergencyStop();

        mContext.events.onSafetyAlert.emit(reason);
        mSM.transition<EmergencyState>();
        mContext.lastError.clear();
        return {};
    }

    /**
     * @brief Emergency -> Preflight.
     *
     * resetEmergencyStop() disables EmergencyLandProfile (if active) — FM Entails
     * cascade auto-cleans MotorMix/ESC — then clears the EmergencyStop Preempts latch.
     * PreflightState::on_entry() calls leaveArmedConfiguration() as belt-and-suspenders.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestReset()
    {
        if (!isEmergency()) return reject("reset", "must be in Emergency state");
        auto clear = mContext.subsystems.resetEmergencyStop();
        if (!clear) return reject("reset", "failed to clear EmergencyStop: " + clear.error());
        mSM.transition<PreflightState>();
        mContext.lastError.clear();
        return {};
    }

private:
    VehicleContext    mContext; // MUST be before mSM — SM holds a Context& bound in ctor
    DroneStateMachine mSM;

    fat_p::Expected<void, std::string> reject(std::string_view cmd, std::string_view reason)
    {
        std::string msg = std::string(cmd) + " rejected: " + std::string(reason);
        mContext.lastError = msg;
        mContext.events.onTransitionRejected.emit(cmd, reason);
        return fat_p::unexpected(std::move(msg));
    }
};

} // namespace drone
