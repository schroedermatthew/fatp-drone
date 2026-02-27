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
  related:
    tests:
      - components/VehicleStateMachine/tests/test_VehicleStateMachine.cpp
  hygiene:
    pragma_once: true
    include_guard: false
    defines_total: 0
    defines_unprefixed: 0
    undefs_total: 0
    includes_windows_h: false
*/

/**
 * @file VehicleStateMachine.h
 * @brief Drone vehicle lifecycle state machine built on fat_p::StateMachine.
 *
 * @details
 * Five vehicle states with guard-protected transitions:
 *
 *   Preflight -> Armed    (guard: validateArmingReadiness passes)
 *   Armed     -> Flying   (guard: at least one flight mode active)
 *   Armed     -> Preflight (disarm; always allowed from Armed)
 *   Flying    -> Landing
 *   Landing   -> Armed    (landing complete)
 *   Landing   -> Preflight (disarm after landing)
 *   Any armed/flying/landing -> Emergency (always allowed)
 *   Emergency -> Preflight  (reset after acknowledgement)
 *
 * Guard logic validates SubsystemManager state before calling transition().
 * If the guard fails, Expected<void, std::string> carries the reason.
 *
 * @note Thread-safety: NOT thread-safe. Use from the single control thread.
 */

#include "DroneEvents.h"
#include "Expected.h"
#include "StateMachine.h"
#include "SubsystemManager.h"

#include <array>
#include <stdexcept>
#include <string>

namespace drone
{

// ============================================================================
// Forward declaration of context (needed by state on_entry/on_exit)
// ============================================================================

struct VehicleContext;

// ============================================================================
// Vehicle states
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
// Transition table (StrictTransitionPolicy - invalid transitions are compile errors)
// ============================================================================

using DroneTransitions = std::tuple<
    std::pair<PreflightState, ArmedState>,      ///< arm
    std::pair<ArmedState,     PreflightState>,  ///< disarm
    std::pair<ArmedState,     FlyingState>,     ///< takeoff
    std::pair<FlyingState,    LandingState>,    ///< land
    std::pair<LandingState,   ArmedState>,      ///< landing complete
    std::pair<LandingState,   PreflightState>,  ///< disarm after landing
    std::pair<ArmedState,     EmergencyState>,  ///< emergency from armed
    std::pair<FlyingState,    EmergencyState>,  ///< emergency from flying
    std::pair<LandingState,   EmergencyState>,  ///< emergency from landing
    std::pair<EmergencyState, PreflightState>   ///< reset after emergency
>;

// ============================================================================
// Vehicle context — shared data visible to all state on_entry/on_exit hooks
// ============================================================================

/**
 * @brief Shared context passed to every state on_entry/on_exit hook.
 *
 * Holds references (not pointers) to ensure they are always valid.
 */
struct VehicleContext
{
    SubsystemManager& subsystems;
    drone::events::DroneEventHub& events;
    std::string lastError;   ///< Set by guard failures; cleared on successful transitions
    std::string fromState;   ///< Name of the state being exited (set by on_exit hooks)
};

// ============================================================================
// State on_entry / on_exit inline implementations
// ============================================================================

// PreflightState ----------------------------------------------------------------

inline void PreflightState::on_entry(VehicleContext& ctx)
{
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}

inline void PreflightState::on_exit(VehicleContext& ctx)
{
    ctx.fromState = kName;
}

// ArmedState --------------------------------------------------------------------

inline void ArmedState::on_entry(VehicleContext& ctx)
{
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}

inline void ArmedState::on_exit(VehicleContext& ctx)
{
    ctx.fromState = kName;
}

// FlyingState -------------------------------------------------------------------

inline void FlyingState::on_entry(VehicleContext& ctx)
{
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}

inline void FlyingState::on_exit(VehicleContext& ctx)
{
    ctx.fromState = kName;
}

// LandingState ------------------------------------------------------------------

inline void LandingState::on_entry(VehicleContext& ctx)
{
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}

inline void LandingState::on_exit(VehicleContext& ctx)
{
    ctx.fromState = kName;
}

// EmergencyState ----------------------------------------------------------------

inline void EmergencyState::on_entry(VehicleContext& ctx)
{
    ctx.events.onSafetyAlert.emit("EmergencyState entered");
    ctx.events.onVehicleStateChanged.emit(ctx.fromState, kName);
}

inline void EmergencyState::on_exit(VehicleContext& ctx)
{
    ctx.fromState = kName;
}

// ============================================================================
// State machine type alias
// ============================================================================

using DroneStateMachine = fat_p::StateMachine<
    VehicleContext,           // Context type
    DroneTransitions,         // Explicit transition table
    fat_p::StrictTransitionPolicy,
    fat_p::ThrowingActionPolicy,
    0,                        // InitialIndex = PreflightState
    PreflightState,
    ArmedState,
    FlyingState,
    LandingState,
    EmergencyState
>;

// ============================================================================
// VehicleStateMachine — thin wrapper with guard logic
// ============================================================================

/**
 * @brief Guard-protected wrapper around DroneStateMachine.
 *
 * All public transition methods validate SubsystemManager preconditions before
 * calling mSM.transition<>(). On failure they return an Expected error and
 * emit onTransitionRejected without touching the SM.
 *
 * @note mContext MUST be declared before mSM because StateMachine holds a
 *       reference to Context that is bound in the StateMachine constructor.
 *       C++ member initialization order follows declaration order.
 */
class VehicleStateMachine
{
public:
    /**
     * @brief Constructs the state machine.
     *
     * @param subsystems Reference to the drone's subsystem manager.
     * @param events     Reference to the drone event hub.
     */
    VehicleStateMachine(SubsystemManager& subsystems, drone::events::DroneEventHub& events)
        : mContext{subsystems, events, {}, {}}
        , mSM(mContext) // mContext must be fully initialized before this line
    {
        // on_entry for PreflightState fires in StateMachine's constructor;
        // fromState is empty string at that point (initial entry has no prior state).
    }

    // Non-copyable, non-movable (SM is neither copyable nor movable)
    VehicleStateMachine(const VehicleStateMachine&) = delete;
    VehicleStateMachine& operator=(const VehicleStateMachine&) = delete;
    VehicleStateMachine(VehicleStateMachine&&) = delete;
    VehicleStateMachine& operator=(VehicleStateMachine&&) = delete;

    // -------------------------------------------------------------------------
    // State queries
    // -------------------------------------------------------------------------

    /// @brief Returns the name of the current vehicle state.
    [[nodiscard]] std::string_view currentStateName() const noexcept
    {
        static constexpr std::array<const char*, 5> kNames = {
            PreflightState::kName,
            ArmedState::kName,
            FlyingState::kName,
            LandingState::kName,
            EmergencyState::kName
        };
        return kNames[mSM.currentStateIndex()];
    }

    /// @brief Returns true if the vehicle is currently in Preflight state.
    [[nodiscard]] bool isPreflight()  const noexcept { return mSM.isInState<PreflightState>(); }

    /// @brief Returns true if the vehicle is currently in Armed state.
    [[nodiscard]] bool isArmed()      const noexcept { return mSM.isInState<ArmedState>(); }

    /// @brief Returns true if the vehicle is currently in Flying state.
    [[nodiscard]] bool isFlying()     const noexcept { return mSM.isInState<FlyingState>(); }

    /// @brief Returns true if the vehicle is currently in Landing state.
    [[nodiscard]] bool isLanding()    const noexcept { return mSM.isInState<LandingState>(); }

    /// @brief Returns true if the vehicle is in Emergency state.
    [[nodiscard]] bool isEmergency()  const noexcept { return mSM.isInState<EmergencyState>(); }

    // -------------------------------------------------------------------------
    // Guard-protected transitions
    // -------------------------------------------------------------------------

    /**
     * @brief Requests arming (Preflight -> Armed).
     *
     * Guard: all arm-required subsystems must be enabled.
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestArm()
    {
        if (!isPreflight())
        {
            return reject("arm", "must be in Preflight state");
        }

        auto guard = mContext.subsystems.validateArmingReadiness();
        if (!guard)
        {
            return reject("arm", guard.error());
        }

        mSM.transition<ArmedState>();
        return {};
    }

    /**
     * @brief Requests disarm (Armed -> Preflight).
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestDisarm()
    {
        if (!isArmed())
        {
            return reject("disarm", "must be in Armed state");
        }

        mSM.transition<PreflightState>();
        return {};
    }

    /**
     * @brief Requests takeoff (Armed -> Flying).
     *
     * Guard: at least one flight mode must be active.
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestTakeoff()
    {
        if (!isArmed())
        {
            return reject("takeoff", "must be in Armed state");
        }

        const std::string mode = mContext.subsystems.activeFlightMode();
        if (mode.empty())
        {
            return reject("takeoff", "no flight mode is active - enable Manual, Stabilize, AltHold, PosHold, Autonomous, or RTL");
        }

        mSM.transition<FlyingState>();
        return {};
    }

    /**
     * @brief Requests landing (Flying -> Landing).
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestLand()
    {
        if (!isFlying())
        {
            return reject("land", "must be in Flying state");
        }

        mSM.transition<LandingState>();
        return {};
    }

    /**
     * @brief Signals landing complete (Landing -> Armed).
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestLandingComplete()
    {
        if (!isLanding())
        {
            return reject("landing_complete", "must be in Landing state");
        }

        mSM.transition<ArmedState>();
        return {};
    }

    /**
     * @brief Requests disarm after landing (Landing -> Preflight).
     *
     * @return Expected<void> on success, or error string.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestDisarmAfterLanding()
    {
        if (!isLanding())
        {
            return reject("disarm_after_landing", "must be in Landing state");
        }

        mSM.transition<PreflightState>();
        return {};
    }

    /**
     * @brief Triggers emergency state unconditionally (from Armed, Flying, or Landing).
     *
     * No guard — emergency is always available. Emits onSafetyAlert.
     *
     * @param reason Human-readable description of the emergency trigger.
     * @return Expected<void> on success, or error string if transition is invalid
     *         from the current state (e.g., already in Preflight or Emergency).
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestEmergency(std::string_view reason)
    {
        if (isEmergency() || isPreflight())
        {
            return reject("emergency", "already in terminal state");
        }

        mContext.events.onSafetyAlert.emit(reason);
        mSM.transition<EmergencyState>();
        return {};
    }

    /**
     * @brief Resets from Emergency to Preflight after operator acknowledgement.
     *
     * @return Expected<void> on success, or error string if not in Emergency.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestReset()
    {
        if (!isEmergency())
        {
            return reject("reset", "must be in Emergency state");
        }

        mSM.transition<PreflightState>();
        return {};
    }

private:
    // CRITICAL: mContext must be declared BEFORE mSM.
    // StateMachine holds a Context& reference bound in its constructor.
    // C++ initializes members in declaration order; if mSM came first,
    // the StateMachine constructor would receive an uninitialized mContext reference.
    VehicleContext   mContext; // initialized first
    DroneStateMachine mSM;     // initialized second; binds mContext in ctor

    fat_p::Expected<void, std::string> reject(std::string_view command, std::string_view reason)
    {
        std::string msg = std::string(command) + " rejected: " + std::string(reason);
        mContext.lastError = msg;
        mContext.events.onTransitionRejected.emit(command, reason);
        return fat_p::unexpected(std::move(msg));
    }
};

} // namespace drone
