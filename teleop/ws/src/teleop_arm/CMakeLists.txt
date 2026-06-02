cmake_minimum_required(VERSION 3.0.2)
project(teleop_arm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  teleop_msgs
  tf2
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs teleop_msgs tf2 tf2_geometry_msgs
)

get_filename_component(Z1_TELEOP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(Z1_SDK_DIR "${Z1_TELEOP_ROOT}/unitree_ws/src/z1_sdk")
set(TELEOP_SCRIPTS_DIR "${Z1_TELEOP_ROOT}/teleop/scripts")

if(NOT EXISTS "${Z1_SDK_DIR}/include/unitree_arm_sdk/control/unitreeArm.h")
  message(FATAL_ERROR "z1_sdk not found at ${Z1_SDK_DIR}")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Z1_SDK_DIR}/include
)

link_directories(${Z1_SDK_DIR}/lib)

set(Z1_SDK_LIB "Z1_SDK_x86_64")

add_executable(communication ${TELEOP_SCRIPTS_DIR}/communication.cpp)
target_link_libraries(communication ${catkin_LIBRARIES} pthread)
add_dependencies(communication ${catkin_EXPORTED_TARGETS})

add_executable(master_arm ${TELEOP_SCRIPTS_DIR}/master_arm.cpp)
target_link_libraries(master_arm ${catkin_LIBRARIES} ${Z1_SDK_LIB} pthread)
add_dependencies(master_arm ${catkin_EXPORTED_TARGETS})
set_target_properties(master_arm PROPERTIES
  BUILD_RPATH "${Z1_SDK_DIR}/lib"
  INSTALL_RPATH "${Z1_SDK_DIR}/lib"
)

add_executable(slave_arm ${TELEOP_SCRIPTS_DIR}/slave_arm.cpp)
target_link_libraries(slave_arm ${catkin_LIBRARIES} ${Z1_SDK_LIB} pthread)
add_dependencies(slave_arm ${catkin_EXPORTED_TARGETS})
set_target_properties(slave_arm PROPERTIES
  BUILD_RPATH "${Z1_SDK_DIR}/lib"
  INSTALL_RPATH "${Z1_SDK_DIR}/lib"
)

add_executable(admittance_debug_logger ${TELEOP_SCRIPTS_DIR}/admittance_debug_logger.cpp)
target_link_libraries(admittance_debug_logger ${catkin_LIBRARIES} pthread)
add_dependencies(admittance_debug_logger ${catkin_EXPORTED_TARGETS})

install(TARGETS communication master_arm slave_arm admittance_debug_logger
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
