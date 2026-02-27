cmake_minimum_required(VERSION 3.20)

#
# FATP_META:
#   meta_version: 1
#   component: DroneConsole
#   file_role: build_script
#   path: app/console/CMakeLists.txt
#   layer: Build
#   summary: Build script for the fatp-drone interactive console application.
#   api_stability: in_work
#

add_executable(fatp-drone-console
    main.cpp
)

target_compile_features(fatp-drone-console PRIVATE cxx_std_20)

target_include_directories(fatp-drone-console PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/drone
    ${FATP_INCLUDE_DIR}
)
