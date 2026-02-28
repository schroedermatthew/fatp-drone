cmake_minimum_required(VERSION 3.20)

#
# FATP_META:
#   meta_version: 1
#   component: DroneCore
#   file_role: build_script
#   path: components/DroneCore/CMakeLists.txt
#   layer: Testing
#   summary: Build script for the DroneCore component and its tests.
#   api_stability: in_work
#

add_library(DroneCore INTERFACE)

target_include_directories(DroneCore INTERFACE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/drone
    ${FATP_INCLUDE_DIR}
)

if(BUILD_TESTING)
    add_executable(test_DroneCore
        tests/test_DroneCore.cpp
    )

    target_compile_features(test_DroneCore PRIVATE cxx_std_20)

    target_compile_definitions(test_DroneCore PRIVATE
        ENABLE_TEST_APPLICATION
    )

    target_include_directories(test_DroneCore PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/drone
        ${FATP_INCLUDE_DIR}
    )

    target_link_libraries(test_DroneCore PRIVATE DroneCore)

    add_test(NAME test_DroneCore COMMAND test_DroneCore)
endif()
