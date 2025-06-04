cmake_minimum_required(VERSION 3.10.2)
project(trick_executor_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE ON) # Keep for debugging make commands

# Explicitly set the install prefix
set(CMAKE_INSTALL_PREFIX "/usr/local")
message(STATUS "CMAKE_INSTALL_PREFIX set to: ${CMAKE_INSTALL_PREFIX}")

find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

set(IGNITION_MATH6_SYSTEM_INCLUDE_DIR "/usr/include/ignition/math6")
if(TARGET ignition-math6::ignition-math6)
  get_target_property(IGN_MATH_INTERFACE_INCLUDES ignition-math6::ignition-math6 INTERFACE_INCLUDE_DIRECTORIES)
  if(IGN_MATH_INTERFACE_INCLUDES)
    set(IGNITION_MATH6_SYSTEM_INCLUDE_DIR ${IGN_MATH_INTERFACE_INCLUDES})
    message(STATUS "Using INTERFACE_INCLUDE_DIRECTORIES from ignition-math6::ignition-math6: ${IGNITION_MATH6_SYSTEM_INCLUDE_DIR}")
  else()
    message(WARNING "Target ignition-math6::ignition-math6 found, but INTERFACE_INCLUDE_DIRECTORIES is empty. Using fallback: ${IGNITION_MATH6_SYSTEM_INCLUDE_DIR}")
  endif()
else()
  message(WARNING "Target ignition-math6::ignition-math6 NOT found. Using fallback include: ${IGNITION_MATH6_SYSTEM_INCLUDE_DIR}")
endif()

add_library(${PROJECT_NAME} SHARED
  src/TrickExecutorPlugin.cc
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/local/include
    ${GAZEBO_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${IGNITION_MATH6_SYSTEM_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${GAZEBO_LIBRARIES}
    ignition-math6::ignition-math6
    ${ZMQ_LIBRARIES}
)

# Explicitly define the full path to avoid issues with variable expansion influenced by other CMake scripts.
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION "/usr/local/lib/gazebo-11/plugins"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE  # Sets rwxr-xr-x
)

message(STATUS "--- CMakeLists.txt Configuration Summary for ${PROJECT_NAME} ---")
get_target_property(TARGET_INCLUDES ${PROJECT_NAME} INCLUDE_DIRECTORIES)
message(STATUS "Target ${PROJECT_NAME} COMPILED_INCLUDE_DIRECTORIES: ${TARGET_INCLUDES}")
if(TARGET ignition-math6::ignition-math6)
  get_target_property(IGN_MATH_INTERFACE_INCLUDES ignition-math6::ignition-math6 INTERFACE_INCLUDE_DIRECTORIES)
  message(STATUS "ignition-math6::ignition-math6 REPORTED INTERFACE_INCLUDE_DIRECTORIES: ${IGN_MATH_INTERFACE_INCLUDES}")
endif()
message(STATUS "--- End of CMakeLists.txt ---")