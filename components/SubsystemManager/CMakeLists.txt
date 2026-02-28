cmake_minimum_required(VERSION 3.20)

#
# FATP_META:
#   meta_version: 1
#   component: SubsystemManager
#   file_role: build_script
#   path: components/SubsystemManager/CMakeLists.txt
#   layer: Testing
#   summary: Build script for the SubsystemManager component and its tests.
#   api_stability: in_work
#

add_library(SubsystemManager INTERFACE)

target_include_directories(SubsystemManager INTERFACE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/drone
    ${FATP_INCLUDE_DIR}
)

if(BUILD_TESTING)
    add_executable(test_SubsystemManager
        tests/test_SubsystemManager.cpp
    )

    target_compile_features(test_SubsystemManager PRIVATE cxx_std_20)

    target_compile_definitions(test_SubsystemManager PRIVATE
        ENABLE_TEST_APPLICATION
    )

    target_include_directories(test_SubsystemManager PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/drone
        ${FATP_INCLUDE_DIR}
    )

    target_link_libraries(test_SubsystemManager PRIVATE SubsystemManager)

    add_test(NAME test_SubsystemManager COMMAND test_SubsystemManager)
endif()
