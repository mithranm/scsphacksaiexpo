cmake_minimum_required(VERSION 3.10)
project(trick_executor_plugin)

# 1) Enforce C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2) Find Gazebo, Ignition Math6, ZeroMQ and nlohmann_json
find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(nlohmann_json 3.2.0 REQUIRED)

# 3) Plugin sources
add_library(${PROJECT_NAME} SHARED
  src/TrickExecutorPlugin.cc
)

# 4) Include directories
target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GAZEBO_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${IGNITION-MATH6_INCLUDE_DIRS}      # pulled in by ignition-math6::ignition-math6
    ${nlohmann_json_INCLUDE_DIRS}        # pulled in by nlohmann_json::nlohmann_json
)

# 5) Link libraries
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${GAZEBO_LIBRARIES}
    ignition-math6::ignition-math6
    ${ZMQ_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# 6) Install into the Gazebo plugin folder
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION "/usr/local/lib/gazebo-11/plugins"
)
