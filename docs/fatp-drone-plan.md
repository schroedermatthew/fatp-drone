# fatp-drone: Detailed Implementation Plan

## Purpose & Scope

**fatp-drone** is a drone subsystem control simulator built entirely on fatp
library components. It demonstrates the fatp ecosystem working as a coherent
whole — not as a game demo but as a realistic control-systems application where
the constraints are hard (wrong combinations cause real problems) and the
architecture decisions are observable.

The application is a console-first ground control simulator. A command loop lets
you arm and disarm subsystems, request flight mode changes, trigger failure
scenarios, and watch the FeatureManager enforce constraints in real time. The
architecture cleanly separates core logic from presentation so that a GUI
(ImGui, Qt, etc.) can be dropped in later without touching the domain layer.

---

## fatp Components Used

| Component | Role |
|---|---|
| `FeatureManager` | Subsystem dependency/conflict graph; the heart of the app |
| `StateMachine` | Vehicle lifecycle: Preflight → Armed → Flying → Landing → Emergency |
| `Signal` | Event bus between subsystems, state machine, telemetry log |
| `Expected<T,E>` | All operation results; no exceptions in domain logic |
| `CircularBuffer` | Rolling telemetry event log |
| `DiagnosticLogger` | Structured log output |
| `JsonLite` | Config save/restore via FeatureManager serialization |
| `enforce` | Precondition checks in command handlers |

---

## Project Structure

```
fatp-drone/
  include/
    drone/
      Subsystems.h            -- Feature name string constants + group enum
      SubsystemManager.h      -- FeatureManager wrapper, drone-domain API
      VehicleStateMachine.h   -- State types + SM typedef + transition guards
      TelemetryLog.h          -- CircularBuffer<TelemetryEvent> + DiagnosticLogger
      DroneEvents.h           -- Signal<> declarations for all event types
      Config.h                -- JSON save/restore helpers
      CommandParser.h         -- Console command interpreter (presentation layer)
  src/
      SubsystemManager.cpp
      VehicleStateMachine.cpp
      TelemetryLog.cpp
      CommandParser.cpp
      main.cpp
  tests/
      test_SubsystemManager.cpp
      test_VehicleStateMachine.cpp
  profiles/
      default.json            -- A realistic starting configuration
      autonomous.json         -- Full autonomous mode ready profile
  CMakeLists.txt
  README.md
```

---

## Layer Architecture

```
┌─────────────────────────────────────────┐
│  Presentation Layer (console/future GUI)│
│  CommandParser.h / main.cpp             │
└──────────────┬──────────────────────────┘
               │  calls
┌──────────────▼──────────────────────────┐
│  Application Layer                      │
│  VehicleController (in main or own file)│
│  - owns SubsystemManager                │
│  - owns VehicleStateMachine             │
│  - owns TelemetryLog                    │
│  - wires Signals                        │
└──────────────┬──────────────────────────┘
               │  uses
┌──────────────▼──────────────────────────┐
│  Domain Layer (zero presentation deps)  │
│  SubsystemManager    VehicleStateMachine│
│  TelemetryLog        DroneEvents        │
└─────────────────────────────────────────┘
               │  built on
┌──────────────▼──────────────────────────┐
│  fatp Library                           │
│  FeatureManager  StateMachine  Signal   │
│  Expected  CircularBuffer  JsonLite etc │
└─────────────────────────────────────────┘
```

**Key rule**: `SubsystemManager`, `VehicleStateMachine`, and `TelemetryLog` must
never `#include` anything from the presentation layer and must never write to
stdout/stderr directly. All output goes through Signals or return values.

---

## File-by-File Specification

---

### `Subsystems.h`

String constants for all feature names. Avoids stringly-typed magic strings
scattered through the codebase. Also defines the group enum.

```cpp
namespace drone::subsystems {

// Sensors
inline constexpr const char* kIMU            = "IMU";
inline constexpr const char* kGPS            = "GPS";
inline constexpr const char* kBarometer      = "Barometer";
inline constexpr const char* kCompass        = "Compass";
inline constexpr const char* kOpticalFlow    = "OpticalFlow";
inline constexpr const char* kLidar          = "Lidar";

// Power
inline constexpr const char* kBatteryMonitor = "BatteryMonitor";
inline constexpr const char* kESC            = "ESC";
inline constexpr const char* kMotorMix       = "MotorMix";

// Comms
inline constexpr const char* kRCReceiver     = "RCReceiver";
inline constexpr const char* kTelemetry      = "Telemetry";
inline constexpr const char* kDatalink       = "Datalink";

// Flight Modes (MutuallyExclusive group)
inline constexpr const char* kManual         = "Manual";
inline constexpr const char* kStabilize      = "Stabilize";
inline constexpr const char* kAltHold        = "AltHold";
inline constexpr const char* kPosHold        = "PosHold";
inline constexpr const char* kAutonomous     = "Autonomous";
inline constexpr const char* kRTL            = "RTL";     // Return to Launch

// Safety
inline constexpr const char* kGeofence           = "Geofence";
inline constexpr const char* kFailsafe            = "Failsafe";
inline constexpr const char* kCollisionAvoidance  = "CollisionAvoidance";
inline constexpr const char* kEmergencyStop       = "EmergencyStop";

// Group names
inline constexpr const char* kGroupSensors        = "Sensors";
inline constexpr const char* kGroupPower          = "Power";
inline constexpr const char* kGroupComms          = "Comms";
inline constexpr const char* kGroupFlightModes    = "FlightModes";
inline constexpr const char* kGroupSafety         = "Safety";

} // namespace drone::subsystems
```

---

### `DroneEvents.h`

All inter-component communication happens through these signals. Any component
that needs to react to change connects a slot — including a future GUI.

```cpp
namespace drone::events {

// Fired when any subsystem changes state (featureName, isEnabled)
using SubsystemChanged = fat_p::Signal<void(std::string_view, bool)>;

// Fired when a subsystem enable/disable request fails (featureName, reason)
using SubsystemError = fat_p::Signal<void(std::string_view, std::string_view)>;

// Fired when vehicle state changes (fromStateName, toStateName)
using VehicleStateChanged = fat_p::Signal<void(std::string_view, std::string_view)>;

// Fired when a state transition is rejected (requestedState, reason)
using VehicleTransitionRejected = fat_p::Signal<void(std::string_view, std::string_view)>;

// Fired when a new telemetry event is logged (timestamp_ms, message)
using TelemetryEvent = fat_p::Signal<void(uint64_t, std::string_view)>;

// Fired on any safety-critical event (eventName)
using SafetyAlert = fat_p::Signal<void(std::string_view)>;

// Aggregate event hub — owned by VehicleController, passed by ref to components
struct DroneEventHub {
    SubsystemChanged      onSubsystemChanged;
    SubsystemError        onSubsystemError;
    VehicleStateChanged   onVehicleStateChanged;
    VehicleTransitionRejected onTransitionRejected;
    TelemetryEvent        onTelemetryEvent;
    SafetyAlert           onSafetyAlert;
};

} // namespace drone::events
```

---

### `SubsystemManager.h`

Wraps `FeatureManager` with drone-domain semantics. Responsible for registering
all features, relationships, and groups on construction. Public API is expressed
in drone terms, not FeatureManager terms.

```cpp
class SubsystemManager {
public:
    explicit SubsystemManager(drone::events::DroneEventHub& events);

    // Enable/disable a named subsystem. Returns success or error string.
    fat_p::Expected<void, std::string> enableSubsystem(std::string_view name);
    fat_p::Expected<void, std::string> disableSubsystem(std::string_view name);

    // Query
    bool isEnabled(std::string_view name) const;

    // What subsystems are currently enabled
    std::vector<std::string> enabledSubsystems() const;

    // Check whether the vehicle is ready to arm (required sensors + power up)
    fat_p::Expected<void, std::string> validateArmingReadiness() const;

    // Check whether a specific flight mode can be activated right now
    fat_p::Expected<void, std::string> validateFlightMode(std::string_view mode) const;

    // Config save/restore
    fat_p::Expected<void, std::string> saveConfig(const std::filesystem::path& path) const;
    fat_p::Expected<void, std::string> loadConfig(const std::filesystem::path& path);

    // GraphViz export for documentation/debugging
    std::string exportDependencyGraph() const;

private:
    void registerSubsystems();
    void registerRelationships();
    void registerGroups();

    fat_p::feature::FeatureManager<> mFeatureManager;
    drone::events::DroneEventHub& mEvents;
};
```

**Dependency graph registered at construction:**

```
// Stabilize mode
Stabilize  Requires  IMU, Barometer

// AltHold mode
AltHold    Requires  Stabilize        (chain: AltHold → Stabilize → IMU + Baro)
AltHold    Requires  Barometer

// PosHold mode
PosHold    Requires  AltHold          (chain adds GPS)
PosHold    Requires  GPS

// Autonomous mode
Autonomous Requires  PosHold
Autonomous Requires  Datalink
Autonomous Requires  CollisionAvoidance
Autonomous Implies   CollisionAvoidance   // auto-enables CA when Autonomous enabled

// RTL
RTL        Requires  GPS
RTL        Requires  AltHold

// Power chain
MotorMix   Requires  ESC
ESC        Requires  BatteryMonitor

// Safety
Failsafe   Requires  BatteryMonitor
Failsafe   Requires  RCReceiver

// MutuallyExclusive flight modes
// All flight modes are in a MutuallyExclusive group:
// Manual, Stabilize, AltHold, PosHold, Autonomous, RTL

// EmergencyStop: conflicts with all flight modes (hard stop)
EmergencyStop Conflicts Manual
EmergencyStop Conflicts Stabilize
EmergencyStop Conflicts AltHold
EmergencyStop Conflicts PosHold
EmergencyStop Conflicts Autonomous
EmergencyStop Conflicts RTL
```

The FeatureManager handles cascade enabling automatically via Implies, and
rejects conflicting combinations via Conflicts/MutuallyExclusive. All of this
behavior comes for free — SubsystemManager just registers the graph.

---

### `VehicleStateMachine.h`

Five vehicle states modeled as C++ types. The SM context holds a reference to
`SubsystemManager` so transition guards can query readiness.

```cpp
// Context: shared data for all states
struct VehicleContext {
    SubsystemManager& subsystems;
    drone::events::DroneEventHub& events;
    std::string lastError;
};

// State types — entry/exit actions fire signals
struct PreflightState  { void on_entry(VehicleContext&); void on_exit(VehicleContext&); };
struct ArmedState      { void on_entry(VehicleContext&); void on_exit(VehicleContext&); };
struct FlyingState     { void on_entry(VehicleContext&); void on_exit(VehicleContext&); };
struct LandingState    { void on_entry(VehicleContext&); void on_exit(VehicleContext&); };
struct EmergencyState  { void on_entry(VehicleContext&); void on_exit(VehicleContext&); };

// Allowed transitions
using DroneTransitions = std::tuple<
    std::pair<PreflightState,  ArmedState>,      // arm
    std::pair<ArmedState,      PreflightState>,  // disarm (safe)
    std::pair<ArmedState,      FlyingState>,     // takeoff
    std::pair<FlyingState,     LandingState>,    // land
    std::pair<LandingState,    ArmedState>,      // landing complete
    std::pair<LandingState,    PreflightState>,  // disarm after landing
    // Emergency is reachable from anywhere
    std::pair<ArmedState,      EmergencyState>,
    std::pair<FlyingState,     EmergencyState>,
    std::pair<LandingState,    EmergencyState>,
    std::pair<EmergencyState,  PreflightState>   // reset after emergency
>;

using DroneStateMachine = fat_p::StateMachine<
    VehicleContext,
    DroneTransitions,
    fat_p::StrictTransitionPolicy,
    fat_p::ThrowingActionPolicy,
    0, // initial state index = PreflightState
    PreflightState, ArmedState, FlyingState, LandingState, EmergencyState
>;

// VehicleStateMachine: thin wrapper with guard logic
class VehicleStateMachine {
public:
    explicit VehicleStateMachine(VehicleContext& ctx);

    // Guard-protected transitions — validate subsystem state before allowing
    fat_p::Expected<void, std::string> requestArm();
    fat_p::Expected<void, std::string> requestDisarm();
    fat_p::Expected<void, std::string> requestTakeoff();
    fat_p::Expected<void, std::string> requestLand();
    fat_p::Expected<void, std::string> requestEmergency();
    fat_p::Expected<void, std::string> requestReset();

    std::string currentStateName() const;
    bool isArmed() const;
    bool isFlying() const;
    bool isEmergency() const;

private:
    DroneStateMachine mSM;
};
```

**Guard logic examples:**

- `requestArm()`: calls `subsystems.validateArmingReadiness()`. If it fails,
  returns the error without calling `mSM.transition<ArmedState>()`.
- `requestTakeoff()`: checks that at least one flight mode is enabled.
- `requestEmergency()`: no guards — always succeeds, fires `SafetyAlert`.

---

### `TelemetryLog.h`

Rolling event log. Consumes Signals from `DroneEventHub` and appends events to
a circular buffer. Can dump the log as a formatted string or JSON.

```cpp
struct TelemetryEntry {
    uint64_t    timestamp_ms;
    std::string source;    // "Subsystem" | "Vehicle" | "Safety"
    std::string event;
    std::string detail;
};

class TelemetryLog {
public:
    static constexpr size_t kDefaultCapacity = 512;

    TelemetryLog(drone::events::DroneEventHub& events,
                 size_t capacity = kDefaultCapacity);

    // Query
    std::vector<TelemetryEntry> recent(size_t n) const;
    std::vector<TelemetryEntry> all() const;
    std::string formatTail(size_t n) const;  // human-readable last n entries

    // Dump full log as JSON
    std::string toJson() const;

    void clear();

private:
    void onSubsystemChanged(std::string_view name, bool enabled);
    void onVehicleStateChanged(std::string_view from, std::string_view to);
    void onSafetyAlert(std::string_view event);

    fat_p::CircularBuffer<TelemetryEntry> mBuffer;
    std::vector<fat_p::ScopedConnection>  mConnections; // RAII signal connections
    uint64_t mStartMs;  // epoch offset for relative timestamps
};
```

---

### `Config.h`

Thin wrapper around `FeatureManager::toJson()` / `fromJson()`. Adds vehicle
metadata (profile name, timestamp, version).

```cpp
namespace drone::config {

struct ProfileMetadata {
    std::string profileName;
    std::string createdAt;  // ISO 8601
    int         version = 1;
};

fat_p::Expected<void, std::string>
saveProfile(const SubsystemManager& sm,
            const ProfileMetadata& meta,
            const std::filesystem::path& path);

fat_p::Expected<void, std::string>
loadProfile(SubsystemManager& sm,
            const std::filesystem::path& path);

} // namespace drone::config
```

---

### `CommandParser.h`

The only file that touches stdout. Parses command strings and calls into
`VehicleStateMachine` and `SubsystemManager`. Returns a result string for
display. This is the boundary between the console frontend and domain logic.

```
Commands:
  enable  <subsystem>       -- enable a named subsystem
  disable <subsystem>       -- disable a named subsystem
  status                    -- show all subsystem states + vehicle state
  arm                       -- request arm transition
  disarm                    -- request disarm transition
  takeoff                   -- request takeoff transition
  land                      -- request landing transition
  emergency                 -- trigger emergency stop
  reset                     -- reset from emergency to preflight
  log [n]                   -- show last n telemetry entries (default 10)
  save <filename>           -- save current profile to JSON
  load <filename>           -- load profile from JSON
  graph                     -- export GraphViz DOT to stdout
  help                      -- show command list
  quit                      -- exit
```

Each command returns a `fat_p::Expected<std::string, std::string>` — success
string or error string — and `main.cpp` decides how to print it. This makes
`CommandParser` testable without stdout capture.

---

### `main.cpp`

Wires everything together. Owns all objects. Runs the command loop.

```
1. Construct DroneEventHub
2. Construct TelemetryLog (connects to hub)
3. Construct SubsystemManager (registers features + relationships + groups)
4. Construct VehicleContext (refs to subsystems + events)
5. Construct VehicleStateMachine (takes context)
6. Construct CommandParser (refs to SM + VSM + log)
7. Print banner
8. Loop: read line → parse → execute → print result
```

The `DroneEventHub` console observer (for live event echo) is connected here,
not in the domain layer. This keeps console output isolated to main.

---

## Dependency Constraint Scenarios to Demonstrate

These scenarios should work correctly automatically — they're the FeatureManager
doing its job, not application-level special cases.

| Scenario | What happens |
|---|---|
| Enable `Autonomous` | Automatically enables `CollisionAvoidance` (Implies). Requires PosHold, which requires AltHold, which requires Stabilize + IMU + Barometer + GPS — all must be enabled first or the request fails with a clear dependency error. |
| Enable `AltHold` with `Manual` active | Rejected — MutuallyExclusive group (FlightModes). Must disable Manual first. |
| Enable `EmergencyStop` while `Autonomous` is active | Conflicts with all flight modes — Autonomous (and its cascade) are blocked. |
| Arm with no power subsystems | `validateArmingReadiness()` rejects it before the SM transition is attempted. |
| Save profile while Autonomous is active | JSON captures full feature graph. Load restores it — including Implies cascade. |
| Disable `GPS` while `PosHold` is active | `PosHold` Requires GPS — disabling GPS should fail (can't remove a dependency while dependent is enabled). FeatureManager handles this. |

---

## CMakeLists.txt Structure

```cmake
cmake_minimum_required(VERSION 3.20)
project(fatp-drone CXX)
set(CMAKE_CXX_STANDARD 20)

# fatp include path (adjust to match your layout)
set(FATP_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../include" CACHE PATH "fatp headers")

add_executable(fatp-drone
    src/SubsystemManager.cpp
    src/VehicleStateMachine.cpp
    src/TelemetryLog.cpp
    src/CommandParser.cpp
    src/main.cpp
)

target_include_directories(fatp-drone
    PRIVATE
    ${FATP_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

# Tests
add_executable(fatp-drone-tests
    tests/test_SubsystemManager.cpp
    tests/test_VehicleStateMachine.cpp
)
target_include_directories(fatp-drone-tests
    PRIVATE
    ${FATP_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)
```

Platform-specific: no platform ifdefs needed at this layer. fatp's internals
handle the platform differences. CMake's default generator handles Windows/Linux.

---

## Implementation Order

1. **`Subsystems.h`** — 15 minutes. Just constants. Gets the vocabulary locked in.

2. **`DroneEvents.h`** — 20 minutes. Define all Signal types and DroneEventHub.

3. **`SubsystemManager.h/.cpp`** — The core. ~2 hours.
   - Start with `registerSubsystems()`: add all features with no relationships.
   - Test: can enable/disable features independently.
   - Add `registerRelationships()`: Requires, Implies, Conflicts, MutuallyExclusive.
   - Test: Autonomous cascade, MutuallyExclusive rejection, EmergencyStop conflicts.
   - Add `validateArmingReadiness()` and `validateFlightMode()`.
   - Wire Signals on enable/disable results.
   - Add `saveConfig()` / `loadConfig()`.

4. **`TelemetryLog.h/.cpp`** — 45 minutes. Mostly signal plumbing and formatting.

5. **`VehicleStateMachine.h/.cpp`** — 1 hour.
   - Define state types with entry/exit hooks.
   - Define transition list.
   - Implement guard methods.

6. **`CommandParser.h/.cpp`** — 45 minutes. Parse strings, call domain API.

7. **`main.cpp`** — 30 minutes. Wire, loop, format output.

8. **`test_SubsystemManager.cpp`** — 1 hour.
   - Test each dependency constraint scenario listed above.
   - Test save/load round-trip.

9. **`test_VehicleStateMachine.cpp`** — 45 minutes.
   - Test invalid transitions are rejected.
   - Test guard failures return Expected errors.
   - Test emergency is always reachable.

10. **Profile JSONs** — 20 minutes. Hand-craft a default and a full-autonomous profile.

11. **README.md** — Document the architecture, build steps, command reference, and dependency graph.

---

## What This Demonstrates About fatp

- **FeatureManager**: Handles a real, non-trivial constraint graph. The flight
  mode mutual exclusion, the Autonomous → CollisionAvoidance implication, and
  the EmergencyStop conflicts are exactly the kinds of relationships the
  component was designed for. The application code doesn't implement any of
  this logic — it just declares the graph.

- **StateMachine**: Compile-time transition enforcement. The vehicle cannot
  jump from Preflight to Flying. The transition matrix is a compile-time
  artifact; invalid transitions don't exist at runtime.

- **Signal**: Clean decoupling. The TelemetryLog doesn't know about
  SubsystemManager. SubsystemManager doesn't know about TelemetryLog. They
  communicate through the event hub. A GUI attaches to the same hub.

- **Expected**: No exceptions in the domain layer. Every operation that can
  fail returns `Expected<void, std::string>`. The presentation layer decides
  what to do with failures.

- **JsonLite**: Config save/restore works at the FeatureManager level. The
  application doesn't write any serialization code — it just calls
  `saveConfig()` / `loadConfig()`.

---

## Open Questions Before Coding

1. **fatp include path convention**: Using `<fat_p/FeatureManager.h>` or flat
   `<FeatureManager.h>`? Confirm the include layout from the main project.

2. **CircularBuffer API**: Confirm the push/iterate interface before
   `TelemetryLog` is implemented.

3. **FeatureManager thread-safety policy**: Single-threaded is fine for the
   console app. If we later add a simulated sensor thread, we'll need
   `MutexPolicy`. Decide at construction time.

4. **FeatureManager disable cascade**: When you disable a feature that other
   features Require, does the manager automatically disable dependents or
   reject the operation? This determines the behavior of "disable GPS while
   PosHold is active" — need to verify from tests or source.
