cmake_minimum_required(VERSION 3.20)

#
# FATP_META:
#   meta_version: 1
#   component: DroneProject
#   file_role: build_script
#   path: CMakeLists.txt
#   layer: Build
#   summary: Root CMake configuration for the fatp-drone simulator project.
#   api_stability: in_work
#

project(fatp-drone
    VERSION 0.1.0
    DESCRIPTION "Drone subsystem simulator demonstrating the Fat-P library ecosystem"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── FATP_INCLUDE_DIR ──────────────────────────────────────────────────────────
# Fat-P headers are a FLAT collection of .h files — no subdirectory, no prefix.
# All includes are written as: #include "FeatureManager.h" (not fat_p/...)
#
# In the Claude project environment, the Fat-P headers live at /mnt/project/:
#   cmake -DFATP_INCLUDE_DIR=/mnt/project ..
#
# For a local checkout, set this to whichever directory contains FeatureManager.h.
if(NOT DEFINED FATP_INCLUDE_DIR)
    set(FATP_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../fat-p-headers"
        CACHE PATH "Directory containing Fat-P .h files (flat layout, no subdirectory)")
endif()

if(NOT EXISTS "${FATP_INCLUDE_DIR}/FeatureManager.h")
    message(WARNING
        "FATP_INCLUDE_DIR='${FATP_INCLUDE_DIR}' does not contain FeatureManager.h. "
        "Pass -DFATP_INCLUDE_DIR=<dir> pointing at the flat Fat-P header directory. "
        "In the Claude project environment use: -DFATP_INCLUDE_DIR=/mnt/project")
endif()

# ── Build options ─────────────────────────────────────────────────────────────

option(BUILD_TESTING "Build test executables" ON)

if(BUILD_TESTING)
    enable_testing()
endif()

# ── Components ────────────────────────────────────────────────────────────────

add_subdirectory(components/SubsystemManager)
add_subdirectory(components/VehicleStateMachine)
add_subdirectory(components/TelemetryLog)
add_subdirectory(components/DroneCore)

# ── Console application ───────────────────────────────────────────────────────

add_subdirectory(app/console)
