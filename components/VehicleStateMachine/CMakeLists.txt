cmake_minimum_required(VERSION 3.20)

#
# FATP_META:
#   meta_version: 1
#   component: VehicleStateMachine
#   file_role: build_script
#   path: components/VehicleStateMachine/CMakeLists.txt
#   layer: Testing
#   summary: Build script for the VehicleStateMachine component and its tests.
#   api_stability: in_work
#

add_library(VehicleStateMachine INTERFACE)

target_include_directories(VehicleStateMachine INTERFACE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/drone
    ${FATP_INCLUDE_DIR}
)

if(BUILD_TESTING)
    add_executable(test_VehicleStateMachine
        tests/test_VehicleStateMachine.cpp
    )

    target_compile_features(test_VehicleStateMachine PRIVATE cxx_std_20)

    target_compile_definitions(test_VehicleStateMachine PRIVATE
        ENABLE_TEST_APPLICATION
    )

    target_include_directories(test_VehicleStateMachine PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/drone
        ${FATP_INCLUDE_DIR}
    )

    target_link_libraries(test_VehicleStateMachine PRIVATE VehicleStateMachine)

    add_test(NAME test_VehicleStateMachine COMMAND test_VehicleStateMachine)
endif()
