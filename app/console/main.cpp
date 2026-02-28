/**
 * @file main.cpp
 * @brief Entry point for the fatp-drone interactive console simulator.
 */
/*
FATP_META:
  meta_version: 1
  component: DroneConsole
  file_role: source
  path: app/console/main.cpp
  namespace: ""
  layer: Testing
  summary: Interactive console REPL for the fatp-drone simulation.
  api_stability: in_work
  related:
    headers:
      - include/drone/CommandParser.h
      - include/drone/VehicleStateMachine.h
      - include/drone/SubsystemManager.h
      - include/drone/TelemetryLog.h
      - include/drone/DroneEvents.h
  hygiene:
    pragma_once: false
    include_guard: false
    defines_total: 0
    defines_unprefixed: 0
    undefs_total: 0
    includes_windows_h: false
*/

#include "CommandParser.h"
#include "DroneEvents.h"
#include "SubsystemManager.h"
#include "TelemetryLog.h"
#include "VehicleStateMachine.h"

#include <iostream>
#include <string>

// Anonymous namespace: compile-time ANSI escape string constants, no mutable state.
namespace
{

constexpr const char* kReset  = "\033[0m";
constexpr const char* kRed    = "\033[31m";
constexpr const char* kGreen  = "\033[32m";
constexpr const char* kYellow = "\033[33m";
constexpr const char* kCyan   = "\033[36m";

void printBanner()
{
    std::cout
        << kCyan
        << "╔═══════════════════════════════════════╗\n"
           "║   fatp-drone  simulator  v0.1         ║\n"
           "║   Fat-P library ecosystem demo        ║\n"
           "╚═══════════════════════════════════════╝\n"
        << kReset
        << "Type 'help' for available commands.\n\n";
}

void printPrompt(std::string_view stateName)
{
    std::cout << kCyan << "[" << stateName << "]" << kReset << " > " << std::flush;
}

void printResult(const drone::CommandResult& result)
{
    if (result.message.empty()) { return; }

    if (result.success)
        std::cout << kGreen << result.message << kReset << "\n";
    else
        std::cout << kRed << result.message << kReset << "\n";
}

} // anonymous namespace

int main()
{
    drone::events::DroneEventHub hub;
    drone::SubsystemManager      mgr{hub};
    drone::VehicleStateMachine   sm{mgr, hub};
    drone::TelemetryLog<512>     log{hub};
    drone::CommandParser<512>    cmd{mgr, sm, log};

    log.logInfo("console", "session started");

    printBanner();

    std::string line;
    while (true)
    {
        printPrompt(sm.currentStateName());

        if (!std::getline(std::cin, line))
        {
            std::cout << "\n" << kYellow << "EOF — exiting." << kReset << "\n";
            break;
        }

        const auto start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) { continue; }
        line = line.substr(start, line.find_last_not_of(" \t\r\n") - start + 1);

        const drone::CommandResult result = cmd.execute(line);
        printResult(result);

        if (result.quit) { break; }
        std::cout << "\n";
    }

    log.logInfo("console", "session ended");
    return 0;
}
