cmake_minimum_required(VERSION 3.20)

#
# FATP_META:
#   meta_version: 1
#   component: TelemetryLog
#   file_role: build_script
#   path: components/TelemetryLog/CMakeLists.txt
#   layer: Testing
#   summary: Build script for the TelemetryLog component and its tests.
#   api_stability: in_work
#

add_library(TelemetryLog INTERFACE)

target_include_directories(TelemetryLog INTERFACE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/drone
    ${FATP_INCLUDE_DIR}
)

if(BUILD_TESTING)
    add_executable(test_TelemetryLog
        tests/test_TelemetryLog.cpp
    )

    target_compile_features(test_TelemetryLog PRIVATE cxx_std_20)

    target_compile_definitions(test_TelemetryLog PRIVATE
        ENABLE_TEST_APPLICATION
    )

    target_include_directories(test_TelemetryLog PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/drone
        ${FATP_INCLUDE_DIR}
    )

    target_link_libraries(test_TelemetryLog PRIVATE TelemetryLog)

    add_test(NAME test_TelemetryLog COMMAND test_TelemetryLog)
endif()
