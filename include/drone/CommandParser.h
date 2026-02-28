#pragma once

/*
FATP_META:
  meta_version: 1
  component: CommandParser
  file_role: public_header
  path: include/drone/CommandParser.h
  namespace: drone
  layer: Domain
  summary: Console command interpreter for the drone simulation. Presentation/domain boundary.
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
 * @file CommandParser.h
 * @brief Console command interpreter.
 *
 * @details
 * CommandParser is the ONLY file that may produce output strings directly.
 * Domain components (SubsystemManager, VehicleStateMachine, TelemetryLog)
 * never write to stdout; they return Expected<> or emit signals.
 *
 * CommandParser::execute() parses a command string, calls the appropriate
 * domain method, and returns a result string for the console to print.
 * This design makes CommandParser fully testable without stdout capture.
 *
 * Command set:
 * @code
 *   enable  <subsystem>   -- enable a named subsystem
 *   disable <subsystem>   -- disable a named subsystem
 *   status                -- show all subsystem and vehicle state
 *   arm                   -- request arm transition
 *   disarm                -- request disarm transition
 *   takeoff               -- request takeoff transition
 *   land                  -- request land transition
 *   landing_complete      -- signal that landing is finished (Landing -> Armed)
 *   disarm_after_landing  -- disarm directly from landing (Landing -> Preflight)
 *   emergency [reason]    -- trigger emergency stop
 *   reset                 -- reset from Emergency to Preflight
 *   log [n]               -- show last n telemetry entries (default 20)
 *   graph                 -- export GraphViz DOT to stdout
 *   json                  -- export current state as JSON
 *   help                  -- show command list
 *   quit                  -- request application exit
 * @endcode
 */

#include "SubsystemManager.h"
#include "TelemetryLog.h"
#include "VehicleStateMachine.h"

#include <sstream>
#include <string>
#include <string_view>

namespace drone
{

/**
 * @brief Result of a command execution.
 *
 * success: true = normal output; false = error output (different display color)
 * message: the string to display
 * quit:    true = application should exit
 */
struct CommandResult
{
    bool success = true;
    std::string message;
    bool quit = false;
};

/**
 * @brief Parses and executes console commands against the drone domain objects.
 *
 * Holds non-owning references to all domain objects. Those objects must outlive
 * this CommandParser.
 *
 * @tparam LogCapacity Template parameter forwarded to TelemetryLog.
 */
template <std::size_t LogCapacity = 512>
class CommandParser
{
public:
    /**
     * @brief Constructs the command parser.
     *
     * @param subsystems SubsystemManager reference.
     * @param sm         VehicleStateMachine reference.
     * @param log        TelemetryLog reference.
     */
    CommandParser(SubsystemManager& subsystems,
                  VehicleStateMachine& sm,
                  TelemetryLog<LogCapacity>& log)
        : mSubsystems(subsystems)
        , mSM(sm)
        , mLog(log)
    {
    }

    /**
     * @brief Parses and executes a single command line.
     *
     * @param line Raw input line (will be trimmed internally).
     * @return CommandResult with display string and quit flag.
     */
    [[nodiscard]] CommandResult execute(std::string_view line)
    {
        std::string cmd;
        std::string arg;

        // Split first token as command, rest as arg
        auto pos = line.find_first_of(" \t");
        if (pos == std::string_view::npos)
        {
            cmd = std::string(line);
        }
        else
        {
            cmd = std::string(line.substr(0, pos));
            auto argStart = line.find_first_not_of(" \t", pos);
            if (argStart != std::string_view::npos)
            {
                arg = std::string(line.substr(argStart));
            }
        }

        // Normalize to lowercase
        for (char& c : cmd)
        {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }

        if (cmd.empty())
        {
            return {true, {}};
        }

        if (cmd == "enable")  { return cmdEnable(arg); }
        if (cmd == "disable") { return cmdDisable(arg); }
        if (cmd == "status")  { return cmdStatus(); }
        if (cmd == "arm")     { return cmdArm(); }
        if (cmd == "disarm")  { return cmdDisarm(); }
        if (cmd == "takeoff") { return cmdTakeoff(); }
        if (cmd == "land")    { return cmdLand(); }
        if (cmd == "landing_complete")     { return cmdLandingComplete(); }
        if (cmd == "disarm_after_landing") { return cmdDisarmAfterLanding(); }
        if (cmd == "emergency") { return cmdEmergency(arg.empty() ? "operator request" : arg); }
        if (cmd == "reset")   { return cmdReset(); }
        if (cmd == "log")     { return cmdLog(arg); }
        if (cmd == "graph")   { return cmdGraph(); }
        if (cmd == "json")    { return cmdJson(); }
        if (cmd == "help")    { return cmdHelp(); }
        if (cmd == "quit" || cmd == "exit") { return {true, "Goodbye.", true}; }

        return {false, "Unknown command: '" + cmd + "'. Type 'help' for command list."};
    }

    /**
     * @brief Returns the help text block.
     */
    [[nodiscard]] static std::string helpText()
    {
        return
            "Available commands:\n"
            "  enable  <subsystem>   -- enable a named subsystem\n"
            "  disable <subsystem>   -- disable a named subsystem\n"
            "  status                -- show all subsystem and vehicle state\n"
            "  arm                   -- arm the vehicle (Preflight -> Armed)\n"
            "  disarm                -- disarm the vehicle (Armed -> Preflight)\n"
            "  takeoff               -- take off (Armed -> Flying)\n"
            "  land                  -- land (Flying -> Landing)\n"
            "  landing_complete      -- signal landing complete (Landing -> Armed)\n"
            "  disarm_after_landing  -- disarm directly from landing (Landing -> Preflight)\n"
            "  emergency [reason]    -- trigger emergency stop\n"
            "  reset                 -- reset from Emergency to Preflight\n"
            "  log [n]               -- show last n telemetry entries (default 20)\n"
            "  graph                 -- export subsystem graph as GraphViz DOT\n"
            "  json                  -- export current state as JSON\n"
            "  help                  -- show this list\n"
            "  quit                  -- exit\n"
            "\n"
            "Subsystem names:\n"
            "  Sensors:      IMU, GPS, Barometer, Compass, OpticalFlow, Lidar\n"
            "  Power:        BatteryMonitor, ESC, MotorMix\n"
            "  Comms:        RCReceiver, Telemetry, Datalink\n"
            "  FlightModes:  Manual, Stabilize, AltHold, PosHold, Autonomous, RTL\n"
            "  Safety:       Geofence, Failsafe, CollisionAvoidance, EmergencyStop\n";
    }

private:
    SubsystemManager&         mSubsystems;
    VehicleStateMachine&      mSM;
    TelemetryLog<LogCapacity>& mLog;

    // -------------------------------------------------------------------------
    // Command implementations
    // -------------------------------------------------------------------------

    CommandResult cmdEnable(const std::string& name)
    {
        if (name.empty())
        {
            return {false, "Usage: enable <subsystem>"};
        }

        auto res = mSubsystems.enableSubsystem(name);
        if (!res)
        {
            return {false, "Enable failed: " + res.error()};
        }
        return {true, "Enabled: " + name};
    }

    CommandResult cmdDisable(const std::string& name)
    {
        if (name.empty())
        {
            return {false, "Usage: disable <subsystem>"};
        }

        auto res = mSubsystems.disableSubsystem(name);
        if (!res)
        {
            return {false, "Disable failed: " + res.error()};
        }
        return {true, "Disabled: " + name};
    }

    CommandResult cmdStatus()
    {
        std::ostringstream oss;
        oss << "Vehicle state: " << mSM.currentStateName() << "\n\n";

        oss << "Enabled subsystems:\n";
        auto enabled = mSubsystems.enabledSubsystems();
        if (enabled.empty())
        {
            oss << "  (none)\n";
        }
        else
        {
            for (const auto& name : enabled)
            {
                oss << "  " << name << "\n";
            }
        }

        const auto mode = mSubsystems.activeFlightMode();
        if (!mode.empty())
        {
            oss << "\nActive flight mode: " << mode << "\n";
        }

        return {true, oss.str()};
    }

    CommandResult cmdArm()
    {
        auto res = mSM.requestArm();
        if (!res)
        {
            return {false, res.error()};
        }
        return {true, "Armed. Vehicle is in Armed state."};
    }

    CommandResult cmdDisarm()
    {
        auto res = mSM.requestDisarm();
        if (!res)
        {
            return {false, res.error()};
        }
        return {true, "Disarmed. Vehicle is in Preflight state."};
    }

    CommandResult cmdTakeoff()
    {
        auto res = mSM.requestTakeoff();
        if (!res)
        {
            return {false, res.error()};
        }
        return {true, "Takeoff initiated. Vehicle is Flying."};
    }

    CommandResult cmdLand()
    {
        auto res = mSM.requestLand();
        if (!res)
        {
            return {false, res.error()};
        }
        return {true, "Landing initiated."};
    }

    CommandResult cmdLandingComplete()
    {
        auto res = mSM.requestLandingComplete();
        if (!res)
        {
            return {false, res.error()};
        }
        return {true, "Landing complete. Vehicle is Armed."};
    }

    CommandResult cmdDisarmAfterLanding()
    {
        auto res = mSM.requestDisarmAfterLanding();
        if (!res)
        {
            return {false, res.error()};
        }
        return {true, "Disarmed after landing. Vehicle is in Preflight state."};
    }

    CommandResult cmdEmergency(const std::string& reason)
    {
        auto res = mSM.requestEmergency(reason);
        if (!res)
        {
            return {false, res.error()};
        }
        return {true, "EMERGENCY STOP: " + reason};
    }

    CommandResult cmdReset()
    {
        auto res = mSM.requestReset();
        if (!res)
        {
            return {false, res.error()};
        }
        return {true, "Reset complete. Vehicle is in Preflight state."};
    }

    CommandResult cmdLog(const std::string& arg)
    {
        std::size_t n = 20;
        if (!arg.empty())
        {
            try
            {
                n = static_cast<std::size_t>(std::stoul(arg));
            }
            catch (...)
            {
                return {false, "Usage: log [n]  (n must be a positive integer)"};
            }
        }

        return {true, mLog.formatTail(n)};
    }

    CommandResult cmdGraph()
    {
        return {true, mSubsystems.exportDependencyGraph()};
    }

    CommandResult cmdJson()
    {
        return {true, mSubsystems.toJson()};
    }

    CommandResult cmdHelp()
    {
        return {true, helpText()};
    }
};

} // namespace drone
