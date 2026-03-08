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
 *   Preflight -> Armed    (guard: validateArmingReadiness passes, including EmergencyStop check)
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
 * Canonical preflight invariant:
 *   PreflightState::on_entry() always calls ctx.subsystems.restorePreflightInvariant()
 *   before emitting the state change event. This ensures no flight mode, MotorMix, or
 *   ESC is left enabled when the vehicle enters Preflight, regardless of the transition
 *   path (disarm, disarm_after_landing, or emergency reset).
 *
 * @note Thread-safety: NOT thread-safe. Use from the single control thread.
 */

#include "DroneEvents.h"
#include "Expected.h"
#include "StateMachine.h"
#include "SubsystemManager.h"

#include <array>
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
// Vehicle context -- shared data visible to all state on_entry/on_exit hooks
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
    std::string lastError;   ///< Set by guard failures; cleared on successful transitions.
    std::string fromState;   ///< Name of the state being exited (set by on_exit hooks).
};

// ============================================================================
// State on_entry / on_exit inline implementations
// ============================================================================

// PreflightState ----------------------------------------------------------------

inline void PreflightState::on_entry(VehicleContext& ctx)
{
    // Enforce canonical preflight invariant every time we enter Preflight,
    // regardless of transition path (disarm, disarm_after_landing, reset).
    // This guarantees no flight mode, MotorMix, or ESC is left enabled.
    ctx.subsystems.restorePreflightInvariant();
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
// VehicleStateMachine -- thin wrapper with guard logic
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
        // on_entry for PreflightState fires in StateMachine's constructor.
        // fromState is empty string at that point (initial entry has no prior state).
        // restorePreflightInvariant() is called; all features start disabled so it
        // is a no-op on construction.
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
     * Guard: all arm-required subsystems must be enabled and EmergencyStop must
     * not be active (validateArmingReadiness checks both).
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
        mContext.lastError.clear();
        return {};
    }

    /**
     * @brief Requests disarm (Armed -> Preflight).
     *
     * PreflightState::on_entry() will call restorePreflightInvariant() to
     * ensure no flight mode, MotorMix, or ESC is left enabled.
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
        mContext.lastError.clear();
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
            return reject("takeoff",
                "no flight mode is active - enable Manual, Stabilize, AltHold, PosHold, Autonomous, or RTL");
        }

        mSM.transition<FlyingState>();
        mContext.lastError.clear();
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
        mContext.lastError.clear();
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
        mContext.lastError.clear();
        return {};
    }

    /**
     * @brief Requests disarm after landing (Landing -> Preflight).
     *
     * PreflightState::on_entry() will call restorePreflightInvariant() to
     * ensure no flight mode, MotorMix, or ESC is left enabled.
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
        mContext.lastError.clear();
        return {};
    }

    /**
     * @brief Triggers emergency state unconditionally (from Armed, Flying, or Landing).
     *
     * Routes to one of two SubsystemManager paths depending on whether the vehicle
     * is airborne:
     *
     *   - Armed (on ground): calls triggerEmergencyStop() -- full motor kill via
     *     forceExclusive(). Safe because the vehicle is not airborne.
     *
     *   - Flying or Landing (airborne): calls triggerEmergencyLand() -- forceExclusive()
     *     sets the EmergencyStop latch and clears flight modes, then the power chain
     *     (BatteryMonitor, ESC, MotorMix) is re-enabled so motors stay live for a
     *     controlled descent. The operator or autopilot must command the descent;
     *     this method only sets the safety state.
     *
     * In both cases the FM Preempts latch is active after this call, blocking any
     * flight mode re-enable until requestReset() is called.
     *
     * The SM transition proceeds even if the subsystem call returns an error, since
     * safety state takes precedence; errors are surfaced via onSubsystemError.
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

        // Route: airborne states keep motors live; ground state cuts everything.
        if (isFlying() || isLanding())
        {
            (void)mContext.subsystems.triggerEmergencyLand();
        }
        else
        {
            (void)mContext.subsystems.triggerEmergencyStop();
        }

        mContext.events.onSafetyAlert.emit(reason);
        mSM.transition<EmergencyState>();
        mContext.lastError.clear();
        return {};
    }

    /**
     * @brief Resets from Emergency to Preflight after operator acknowledgement.
     *
     * Calls SubsystemManager::resetEmergencyStop() to clear the EmergencyStop latch
     * (FM Preempts latch lifted), then transitions to Preflight. PreflightState::on_entry()
     * calls restorePreflightInvariant() to power down the motor chain that was left
     * live by triggerEmergencyLand().
     *
     * Returns an error if the clear fails so the operator knows the latch was not
     * released and re-arming would still be blocked.
     *
     * @return Expected<void> on success, or error string if not in Emergency or
     *         if the EmergencyStop latch cannot be cleared.
     */
    [[nodiscard]] fat_p::Expected<void, std::string> requestReset()
    {
        if (!isEmergency())
        {
            return reject("reset", "must be in Emergency state");
        }

        auto clear = mContext.subsystems.resetEmergencyStop();
        if (!clear)
        {
            return reject("reset", "failed to clear EmergencyStop: " + clear.error());
        }

        mSM.transition<PreflightState>();
        mContext.lastError.clear();
        return {};
    }

private:
    // CRITICAL: mContext must be declared BEFORE mSM.
    // StateMachine holds a Context& reference bound in its constructor.
    // C++ initializes members in declaration order; if mSM came first,
    // the StateMachine constructor would receive an uninitialized mContext reference.
    VehicleContext    mContext; // initialized first
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
