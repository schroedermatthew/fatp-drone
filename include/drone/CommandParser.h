#pragma once

/*
FATP_META:
  meta_version: 1
  component: CommandParser
  file_role: public_header
  path: include/drone/CommandParser.h
  namespace: drone
  layer: Domain
  summary: Console command interpreter for the drone simulation.
  api_stability: in_work
*/

/**
 * @file CommandParser.h
 * @brief Console command interpreter.
 *
 * Bridges user input to domain methods. Has no subsystem logic of its own.
 *
 * Reserved features — blocked from raw enable/disable:
 *   EmergencyStop      — use 'emergency' / 'reset'
 *   ArmedProfile       — managed by Armed state on_entry/on_exit
 *   EmergencyLandProfile — managed by triggerEmergencyLand / resetEmergencyStop
 */

#include "SubsystemManager.h"
#include "Subsystems.h"
#include "TelemetryLog.h"
#include "VehicleStateMachine.h"

#include <cctype>
#include <sstream>
#include <string>
#include <string_view>

namespace drone
{

struct CommandResult
{
    bool success = true;
    std::string message;
    bool quit = false;
};

template <std::size_t LogCapacity = 512>
class CommandParser
{
public:
    CommandParser(SubsystemManager& subsystems,
                  VehicleStateMachine& sm,
                  TelemetryLog<LogCapacity>& log)
        : mSubsystems(subsystems), mSM(sm), mLog(log)
    {}

    [[nodiscard]] CommandResult execute(std::string_view line)
    {
        // Trim leading whitespace
        auto start = line.find_first_not_of(" \t");
        line = (start != std::string_view::npos) ? line.substr(start) : std::string_view{};

        std::string cmd, arg;
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
                arg = std::string(line.substr(argStart));
        }

        for (char& c : cmd)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        if (cmd.empty())                   return {true, {}};
        if (cmd == "enable")               return cmdEnable(arg);
        if (cmd == "disable")              return cmdDisable(arg);
        if (cmd == "status")               return cmdStatus();
        if (cmd == "arm")                  return cmdArm();
        if (cmd == "disarm")               return cmdDisarm();
        if (cmd == "takeoff")              return cmdTakeoff();
        if (cmd == "land")                 return cmdLand();
        if (cmd == "landing_complete")     return cmdLandingComplete();
        if (cmd == "disarm_after_landing") return cmdDisarmAfterLanding();
        if (cmd == "emergency")            return cmdEmergency(arg.empty() ? "operator request" : arg);
        if (cmd == "reset")                return cmdReset();
        if (cmd == "log")                  return cmdLog(arg);
        if (cmd == "graph")                return cmdGraph();
        if (cmd == "json")                 return cmdJson();
        if (cmd == "help")                 return cmdHelp();
        if (cmd == "quit" || cmd == "exit") return {true, "Goodbye.", true};

        return {false, "Unknown command: '" + cmd + "'. Type 'help' for command list."};
    }

    [[nodiscard]] static std::string helpText()
    {
        return
            "Available commands:\n"
            "  enable  <subsystem>   -- enable a named subsystem\n"
            "  disable <subsystem>   -- disable a named subsystem\n"
            "  status                -- show subsystem and vehicle state\n"
            "  arm                   -- arm the vehicle (Preflight -> Armed)\n"
            "  disarm                -- disarm (Armed -> Preflight)\n"
            "  takeoff               -- take off (Armed -> Flying)\n"
            "  land                  -- land (Flying -> Landing)\n"
            "  landing_complete      -- landing done (Landing -> Armed)\n"
            "  disarm_after_landing  -- disarm from landing (Landing -> Preflight)\n"
            "  emergency [reason]    -- trigger emergency (stop on ground / land if airborne)\n"
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
            "  Safety:       Geofence, Failsafe, CollisionAvoidance\n"
            "                (EmergencyStop managed via emergency/reset commands)\n"
            "  Note: ArmedProfile and EmergencyLandProfile are internal FM features.\n";
    }

private:
    SubsystemManager&          mSubsystems;
    VehicleStateMachine&       mSM;
    TelemetryLog<LogCapacity>& mLog;

    // Features the user may not directly toggle — they are owned by the SM / graph.
    static bool isReserved(const std::string& name)
    {
        using namespace drone::subsystems;
        return name == kEmergencyStop
            || name == kProfileArmed
            || name == kProfileEmergencyLand;
    }

    CommandResult cmdEnable(const std::string& name)
    {
        if (name.empty()) return {false, "Usage: enable <subsystem>"};
        if (isReserved(name))
            return {false, "'" + name + "' is reserved. "
                "Use the 'emergency' command to trigger an emergency stop."};
        auto res = mSubsystems.enableSubsystem(name);
        if (!res) return {false, "Enable failed: " + res.error()};
        return {true, "Enabled: " + name};
    }

    CommandResult cmdDisable(const std::string& name)
    {
        if (name.empty()) return {false, "Usage: disable <subsystem>"};
        if (isReserved(name))
            return {false, "'" + name + "' is reserved. "
                "Use the 'reset' command to clear an active emergency."};
        auto res = mSubsystems.disableSubsystem(name);
        if (!res) return {false, "Disable failed: " + res.error()};
        return {true, "Disabled: " + name};
    }

    CommandResult cmdStatus()
    {
        std::ostringstream oss;
        oss << "Vehicle state: " << mSM.currentStateName() << "\n\n";
        oss << "Enabled subsystems:\n";
        auto enabled = mSubsystems.enabledSubsystems();
        if (enabled.empty()) oss << "  (none)\n";
        else for (const auto& n : enabled) oss << "  " << n << "\n";
        const auto mode = mSubsystems.activeFlightMode();
        if (!mode.empty()) oss << "\nActive flight mode: " << mode << "\n";
        return {true, oss.str()};
    }

    CommandResult cmdArm()
    {
        auto res = mSM.requestArm();
        if (!res) return {false, res.error()};
        return {true, "Armed. Vehicle is in Armed state."};
    }

    CommandResult cmdDisarm()
    {
        auto res = mSM.requestDisarm();
        if (!res) return {false, res.error()};
        return {true, "Disarmed. Vehicle is in Preflight state."};
    }

    CommandResult cmdTakeoff()
    {
        auto res = mSM.requestTakeoff();
        if (!res) return {false, res.error()};
        return {true, "Takeoff initiated. Vehicle is Flying."};
    }

    CommandResult cmdLand()
    {
        auto res = mSM.requestLand();
        if (!res) return {false, res.error()};
        return {true, "Landing initiated."};
    }

    CommandResult cmdLandingComplete()
    {
        auto res = mSM.requestLandingComplete();
        if (!res) return {false, res.error()};
        return {true, "Landing complete. Vehicle is Armed."};
    }

    CommandResult cmdDisarmAfterLanding()
    {
        auto res = mSM.requestDisarmAfterLanding();
        if (!res) return {false, res.error()};
        return {true, "Disarmed after landing. Vehicle is in Preflight state."};
    }

    CommandResult cmdEmergency(const std::string& reason)
    {
        const bool airborne = mSM.isFlying() || mSM.isLanding();
        auto res = mSM.requestEmergency(reason);
        if (!res) return {false, res.error()};
        const std::string prefix = airborne ? "EMERGENCY LAND: " : "EMERGENCY STOP: ";
        return {true, prefix + reason};
    }

    CommandResult cmdReset()
    {
        auto res = mSM.requestReset();
        if (!res) return {false, res.error()};
        return {true, "Reset complete. Vehicle is in Preflight state."};
    }

    CommandResult cmdLog(const std::string& arg)
    {
        std::size_t n = 20;
        if (!arg.empty())
        {
            try { n = static_cast<std::size_t>(std::stoul(arg)); }
            catch (...) { return {false, "Usage: log [n]  (n must be a positive integer)"}; }
        }
        return {true, mLog.formatTail(n)};
    }

    CommandResult cmdGraph() { return {true, mSubsystems.exportDependencyGraph()}; }
    CommandResult cmdJson()  { return {true, mSubsystems.toJson()}; }
    CommandResult cmdHelp()  { return {true, helpText()}; }
};

} // namespace drone
