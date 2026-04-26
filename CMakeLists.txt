cmake_minimum_required(VERSION 3.0.2)
project(can_driver)

set(INNFOS_SDK_DIR "")
set(INNFOS_SDK_CANDIDATES
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/innfos-cpp-sdk"
  "${CMAKE_CURRENT_SOURCE_DIR}/1MINTASCA/innfos-cpp-sdk"
)
foreach(candidate ${INNFOS_SDK_CANDIDATES})
  if(EXISTS "${candidate}")
    set(INNFOS_SDK_DIR "${candidate}")
    break()
  endif()
endforeach()

if(INNFOS_SDK_DIR STREQUAL "")
  message(FATAL_ERROR "Failed to locate innfos-cpp-sdk under lib/ or 1MINTASCA/.")
endif()

set(INNFOS_SDK_INCLUDE_DIR "${INNFOS_SDK_DIR}/sdk/include")
set(INNFOS_SDK_LIB_DIR "${INNFOS_SDK_DIR}/sdk/lib/linux_x86_64")
unset(INNFOS_ACTUATOR_LIB CACHE)
find_library(INNFOS_ACTUATOR_LIB
  NAMES ActuatorController
  PATHS ${INNFOS_SDK_LIB_DIR}
  NO_DEFAULT_PATH
)
if(NOT INNFOS_ACTUATOR_LIB)
  message(FATAL_ERROR "Failed to find libActuatorController.so under ${INNFOS_SDK_LIB_DIR}")
endif()

## Use C++17 to unlock std::clamp 等现代特性
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  controller_manager
  hardware_interface
  joint_limits_interface
  message_generation
  roscpp
  std_msgs
  std_srvs
  urdf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ApplyLimits.srv
  Init.srv
  Shutdown.srv
  Recover.srv
  MotorCommand.srv
  SetZero.srv
  SetZeroLimit.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveMotor.action
  HomeMotor.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES can_driver_transport
  CATKIN_DEPENDS actionlib actionlib_msgs controller_manager hardware_interface joint_limits_interface message_runtime roscpp std_msgs std_srvs urdf
)

# Enable testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${INNFOS_SDK_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(can_driver_transport
  src/CanDriverRuntime.cpp
  src/DeviceManager.cpp
  src/DeviceRuntime.cpp
  src/JointConfigParser.cpp
  src/MtCan.cpp
  src/EyouCan.cpp
  src/DamiaoCan.cpp
  src/InnfosEcbProtocol.cpp
  src/driver_ros_endpoints.cpp
  src/command_gate.cpp
  src/lifecycle_driver_ops.cpp
  src/lifecycle_service_gateway.cpp
  src/motor_action_executor.cpp
  src/motor_maintenance_service.cpp
  src/operational_coordinator.cpp
  src/SocketCanController.cpp
  src/UdpCanTransport.cpp
  src/SafeCommand.cpp
  src/CanDriverHW.cpp
)
add_dependencies(can_driver_transport ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(can_driver_transport
  ${INNFOS_ACTUATOR_LIB}
  pthread
  ${catkin_LIBRARIES}
)
if(NOT INNFOS_SDK_DIR STREQUAL "")
  set_target_properties(can_driver_transport PROPERTIES
    BUILD_RPATH ${INNFOS_SDK_LIB_DIR}
    INSTALL_RPATH ${INNFOS_SDK_LIB_DIR}
  )
endif()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(can_driver_node src/can_driver_node.cpp)
add_dependencies(can_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_driver_node
  can_driver_transport
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/realtime_motor_ui.py
  scripts/mt_motor_interface.py
  scripts/socketcan_scan_ids.py
  scripts/lifecycle_compat_autostart.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_socketcan_controller
    tests/test_socketcan_controller.cpp
  )
  if(TARGET test_socketcan_controller)
    target_include_directories(test_socketcan_controller PRIVATE include ${GTEST_INCLUDE_DIRS})
    # Link against gtest and gtest_main to provide main()
    if (TARGET GTest::GTest)
      target_link_libraries(test_socketcan_controller
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_socketcan_controller
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_udp_can_transport
    tests/test_udp_can_transport.cpp
  )
  if(TARGET test_udp_can_transport)
    target_include_directories(test_udp_can_transport PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_udp_can_transport
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_udp_can_transport
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_safe_command
    tests/test_safe_command.cpp
  )
  if(TARGET test_safe_command)
    target_include_directories(test_safe_command PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_safe_command
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_safe_command
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_joint_config_parser
    tests/test_joint_config_parser.cpp
  )
  if(TARGET test_joint_config_parser)
    target_include_directories(test_joint_config_parser PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_joint_config_parser
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_joint_config_parser
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_device_manager
    tests/test_device_manager.cpp
  )
  if(TARGET test_device_manager)
    target_include_directories(test_device_manager PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_device_manager
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_device_manager
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_device_runtime
    tests/test_device_runtime.cpp
  )
  if(TARGET test_device_runtime)
    target_include_directories(test_device_runtime PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_device_runtime
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_device_runtime
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_limits_enforcement
    tests/test_limits_enforcement.cpp
  )
  if(TARGET test_limits_enforcement)
    target_include_directories(test_limits_enforcement PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_limits_enforcement
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_limits_enforcement
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_mt_can_protocol
    tests/test_mt_can_protocol.cpp
  )
  if(TARGET test_mt_can_protocol)
    target_include_directories(test_mt_can_protocol PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_mt_can_protocol
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_mt_can_protocol
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_damiao_can_protocol
    tests/test_damiao_can_protocol.cpp
  )
  if(TARGET test_damiao_can_protocol)
    target_include_directories(test_damiao_can_protocol PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_damiao_can_protocol
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_damiao_can_protocol
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_refresh_scheduler
    tests/test_refresh_scheduler.cpp
  )
  if(TARGET test_refresh_scheduler)
    target_include_directories(test_refresh_scheduler PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_refresh_scheduler
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_refresh_scheduler
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_eyou_can_protocol
    tests/test_eyou_can_protocol.cpp
  )
  if(TARGET test_eyou_can_protocol)
    target_include_directories(test_eyou_can_protocol PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_eyou_can_protocol
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_eyou_can_protocol
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_eyou_can_csp
    tests/test_eyou_can_csp.cpp
  )
  if(TARGET test_eyou_can_csp)
    target_include_directories(test_eyou_can_csp PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_eyou_can_csp
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_eyou_can_csp
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  add_rostest_gtest(test_can_driver_hw_smoke
    tests/test_can_driver_hw_smoke.test
    tests/test_can_driver_hw_smoke.cpp
  )
  if(TARGET test_can_driver_hw_smoke)
    target_include_directories(test_can_driver_hw_smoke PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_can_driver_hw_smoke
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_can_driver_hw_smoke
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_can_driver_operational_coordinator
    tests/test_operational_coordinator.cpp
  )
  if(TARGET test_can_driver_operational_coordinator)
    target_include_directories(test_can_driver_operational_coordinator PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_can_driver_operational_coordinator
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_can_driver_operational_coordinator
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()

  catkin_add_gtest(test_can_driver_lifecycle_driver_ops
    tests/test_lifecycle_driver_ops.cpp
  )
  if(TARGET test_can_driver_lifecycle_driver_ops)
    target_include_directories(test_can_driver_lifecycle_driver_ops PRIVATE include ${GTEST_INCLUDE_DIRS})
    if (TARGET GTest::GTest)
      target_link_libraries(test_can_driver_lifecycle_driver_ops
        can_driver_transport
        ${catkin_LIBRARIES}
        GTest::GTest GTest::Main
      )
    else()
      target_link_libraries(test_can_driver_lifecycle_driver_ops
        can_driver_transport
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      )
    endif()
  endif()
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
