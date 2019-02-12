cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
# cmake_policy(SET CMP0042 NEW)
# cmake_policy(SET CMP0048 NEW)
# cmake_policy(SET CMP0054 NEW)

if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "install prefix" FORCE)
endif()

message(STATUS "install-prefix: ${CMAKE_INSTALL_PREFIX}")

project(crazyflie_gazebo)
include(GNUInstallDirs)

#######################
## Find Dependencies ##
#######################



# Add search directory for CMake on OS X
list(APPEND CMAKE_MODULE_PATH /usr/local/share/cmake/Modules)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# option(BUILD_GSTREAMER_PLUGIN "enable gstreamer plugin" "OFF")

option(BUILD_ROS_INTERFACE "enable ROS subscriber for motor failure plugin" "ON")

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)
find_package(gazebo REQUIRED)
# find_program(px4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread timer)
find_package(PythonInterp REQUIRED)
# if (BUILD_GSTREAMER_PLUGIN)
#   find_package(GStreamer)
#   if (GSTREAMER_FOUND)
#     if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
#       find_package (Qt4)
#       include (${QT_USE_FILE})
#     endif()
#   endif()
# endif()

pkg_check_modules(OGRE OGRE)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# add_subdirectory( external/OpticalFlow OpticalFlow )
# set( OpticalFlow_LIBS "OpticalFlow" )

# =============================================================================================== #
# ====================================== CF COMMUNICATION LIB =================================== #
# =============================================================================================== #
set (CRAZYFLIE_COMM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/crazyflie_comm/include)
set (CRAZYFLIE_COMM_LIBRARY crazyflie_comm)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/crazyflie_comm)
include_directories(
  ${CRAZYFLIE_COMM_INCLUDE_DIR}
)
list(APPEND targets_to_install crazyflie_comm)
#================================================================================================ #


# # for ROS subscribers and publishers
# if (BUILD_ROS_INTERFACE)
#   find_package(roscpp REQUIRED)
#   find_package(mavros REQUIRED)
#   find_package(mavros_msgs REQUIRED)
#   find_package(geometry_msgs REQUIRED)
#   find_package(sensor_msgs REQUIRED)
#   find_package(mav_msgs REQUIRED)
#   find_package(std_msgs REQUIRED)
# endif()

# find MAVLink
# find_package(MAVLink)

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message("Gazebo version: ${GAZEBO_VERSION}")
endif()

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost 1.40 COMPONENTS system thread timer REQUIRED )

# ============================================== #
# ==================== CATKIN ================== #
# ============================================== #

################################################
## Declare ROS messages, services and actions ##
################################################



find_package(catkin REQUIRED COMPONENTS
  message_generation
  cmake_modules
  cv_bridge
  geometry_msgs
  sensor_msgs
  mav_msgs
  roscpp
  std_srvs
  tf
  crazyflie_driver
)

add_message_files(
  FILES
  WindSpeed.msg
)

# declare crazyflie python module
catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
   std_msgs
   geometry_msgs
   crazyflie_driver
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs sensor_msgs roscpp std_srvs std_msgs tf message_runtime crazyflie_driver
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS})

###########
## Build ##
###########

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -std=c99 -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
# -Wno-deprecated-declarations: Shut up warnings about std::binder1st, std::binder2nd.

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${OpticalFlow_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Boost_INCLUDE_DIR}
  # Workaround for Eigen3
  ${Boost_INCLUDE_DIR}/eigen3
  ${EIGEN3_INCLUDE_DIRS}/eigen3
  ${OGRE_INCLUDE_DIRS}
  # workaround for: "fatal error: OgrePagedWorldSection.h: No such file or directory"
  ${OGRE_INCLUDE_DIRS}/Paging
  # Workaround for OGRE include dirs on Mac OS
  # TODO: remove these hardcoded includes now that we have
  /usr/local/include/OGRE
  /usr/local/include/OGRE/Paging
  ${MAVLINK_INCLUDE_DIRS}
  )

# if (GSTREAMER_FOUND)
#   include_directories(
#     ${GSTREAMER_INCLUDE_DIRS}
#   )
# endif()

link_libraries(
  ${PROTOBUF_LIBRARY}
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  )

# if (GSTREAMER_FOUND)
#   link_libraries(
#     ${GSTREAMER_LIBRARIES}
#   )
# endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_LIBRARY_DIRS}
  )

#--------------------------#
# Generation of SDF models #
#--------------------------#

# sudo apt-get install ros-kinetic-pr2-simulator

# set(enable_wind "false")

# set(rotors_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/models/rotors_description")
# set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")

# set(uri "INADDR_ANY://19950")
# set(enable_ground_truth "true")
# set(color_prop_front "Blue")
# set(color_prop_back "Red")
# set(namespace "/")

# set(enable_logging "true")
# set(use_ros_time "true")
# set(enable_logging_imu "true")
# set(enable_logging_packets "false")
# set(enable_logging_magnetic_field "false")
# set(enable_logging_temperature "false")
# set(enable_logging_pressure "false")
# set(enable_logging_battery "false")
# set(enable_parameters "true")

# add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/crazyflie/crazyflie.sdf
#   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#   COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/crazyflie/crazyflie.sdf
#   COMMAND ${PYTHON_EXECUTABLE} ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/crazyflie_base.urdf  ${rotors_description_dir}/urdf/crazyflie_base.xacro namespace:=${namespace} enable_wind:=${enable_wind} enable_ground_truth:=${enable_ground_truth} color_prop_front:=${color_prop_front} color_prop_back:=${color_prop_back} rotors_description_dir:=${rotors_description_dir}
#   COMMAND gz sdf -p  ${rotors_description_dir}/urdf/crazyflie_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/crazyflie/crazyflie.sdf
#   COMMAND rm -f ${rotors_description_dir}/urdf/crazyflie_base.urdf
#   DEPENDS ${rotors_description_dir}/urdf/crazyflie.xacro
#   DEPENDS ${rotors_description_dir}/urdf/crazyflie_base.xacro
#   DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
#   )
# add_custom_target(crazyflie_sf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/crazyflie/crazyflie.sdf)

# set(nbQuads "1")
# set(cfPrefix "cf")
# set(uri "INADDR_ANY://19950")
# set(firstIndex "1")

# add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/cfHandler/cfHandler.sdf
#   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#   COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/cfHandler/cfHandler.sdf
#   COMMAND ${PYTHON_EXECUTABLE} ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/cfHandler_base.urdf  ${rotors_description_dir}/urdf/cfHandler_base.xacro nbQuads:=${nbQuads} cfPrefix:=${cfPrefix} enable_parameters:=${enable_parameters} enable_logging:=${enable_logging} uri:=${uri} firstIndex:=${firstIndex} rotors_description_dir:=${rotors_description_dir} use_ros_time:=${use_ros_time} enable_logging_battery:=${enable_logging_battery} enable_logging_pressure:=${enable_logging_pressure} enable_logging_temperature:=${enable_logging_temperature} enable_logging_imu:=${enable_logging_imu} enable_logging_packets:=${enable_logging_packets} enable_logging_magnetic_field:=${enable_logging_magnetic_field}
#   COMMAND gz sdf -p  ${rotors_description_dir}/urdf/cfHandler_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/cfHandler/cfHandler.sdf
#   COMMAND rm -f ${rotors_description_dir}/urdf/cfHandler_base.urdf
#   DEPENDS ${rotors_description_dir}/urdf/cfHandler_base.xacro
#   DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
#   )
# add_custom_target(cfHandler_sf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/cfHandler/cfHandler.sdf)

# set(poseTopic "/cf1/pose")

# add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/crazyflie_ghost/crazyflie_ghost.sdf
#   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#   COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/crazyflie_ghost/crazyflie_ghost.sdf
#   COMMAND ${PYTHON_EXECUTABLE} ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/crazyflie_ghost_base.urdf  ${rotors_description_dir}/urdf/crazyflie_ghost_base.xacro poseTopic:=${poseTopic} namespace:=${namespace} rotors_description_dir:=${rotors_description_dir} color_prop_front:=${color_prop_front} color_prop_back:=${color_prop_back}
#   COMMAND gz sdf -p  ${rotors_description_dir}/urdf/crazyflie_ghost_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/crazyflie_ghost/crazyflie_ghost.sdf
#   COMMAND rm -f ${rotors_description_dir}/urdf/crazyflie_ghost_base.urdf
#   DEPENDS ${rotors_description_dir}/urdf/crazyflie_ghost_base.xacro
#   DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
#   )
# add_custom_target(crazyflie_ghost_sf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/crazyflie_ghost/crazyflie_ghost.sdf)


#-----------#
# Functions #
#-----------#
function(glob_generate target file_glob)
  file(GLOB_RECURSE glob_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${file_glob})
	set(gen_files)
	foreach(glob_file ${glob_files})
		string(REGEX REPLACE "\\.[^.]*$" "" file_name ${glob_file})
		string(REGEX MATCH "[^.]*$" file_ext ${glob_file})
		get_filename_component(file_dir ${glob_file} DIRECTORY)
		set(in_file ${CMAKE_CURRENT_SOURCE_DIR}/${glob_file})
		file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${file_dir})
		set(out_file ${CMAKE_CURRENT_SOURCE_DIR}/${file_name})
		string(REGEX REPLACE ".sdf" "-gen.sdf" out_file ${out_file})
		if (${file_ext} STREQUAL "jinja")
			add_custom_command(OUTPUT ${out_file}
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/jinja_gen.py
        ${in_file} ${CMAKE_CURRENT_SOURCE_DIR}
				DEPENDS ${in_file}
				VERBATIM
				)
			list(APPEND gen_files_${target} ${out_file})
		endif()
	endforeach()
	add_custom_target(${target} ALL DEPENDS ${gen_files_${target}})
endfunction()

glob_generate(models_gen ${CMAKE_CURRENT_SOURCE_DIR}/models/*.jinja)

#--------------------#
# Message Generation #
#--------------------#

# set (msgs
#   msgs/Float.proto
#   msgs/Int32.proto
#   msgs/SensorImu.proto
#   msgs/opticalFlow.proto
#   msgs/lidar.proto
#   msgs/CommandMotorSpeed.proto
#   msgs/MotorSpeed.proto
#   #msgs/Wind.proto
#   msgs/sonarSens.proto
#   msgs/irlock.proto
#   msgs/SITLGps.proto
#   msgs/Groundtruth.proto
#   msgs/odom.proto
# )
file(GLOB msgs msgs/*.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(mav_msgs SHARED ${PROTO_SRCS})
target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY} gazebo_msgs)

#---------#
# Plugins #
#---------#


link_libraries(mav_msgs)


add_library(rotors_gazebo_ros_interface_plugin SHARED src/gazebo_ros_interface_plugin.cpp)
target_link_libraries(rotors_gazebo_ros_interface_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_ros_interface_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_odometry_plugin SHARED src/gazebo_odometry_plugin.cpp)
target_link_libraries(rotors_gazebo_odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(rotors_gazebo_odometry_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
target_link_libraries(rotors_gazebo_controller_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_controller_interface ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
target_link_libraries(rotors_gazebo_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_motor_model ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
target_link_libraries(rotors_gazebo_multirotor_base_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_multirotor_base_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
target_link_libraries(rotors_gazebo_imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_imu_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_pressure_plugin SHARED src/gazebo_pressure_plugin.cpp)
target_link_libraries(rotors_gazebo_pressure_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_pressure_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_magnetometer_plugin SHARED src/gazebo_magnetometer_plugin.cpp)
target_link_libraries(rotors_gazebo_magnetometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_magnetometer_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_lps_plugin SHARED src/gazebo_lps_plugin.cpp)
target_link_libraries(rotors_gazebo_lps_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_lps_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_wind_plugin SHARED src/gazebo_wind_plugin.cpp)
target_link_libraries(rotors_gazebo_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_wind_plugin ${catkin_EXPORTED_TARGETS})

add_library(gazebo_sonar_plugin SHARED src/gazebo_sonar_plugin.cpp)

add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)

set(plugins
  rotors_gazebo_ros_interface_plugin
  rotors_gazebo_odometry_plugin
  rotors_gazebo_controller_interface
  rotors_gazebo_motor_model
  rotors_gazebo_multirotor_base_plugin
  rotors_gazebo_imu_plugin
  rotors_gazebo_pressure_plugin
  rotors_gazebo_magnetometer_plugin
  rotors_gazebo_lps_plugin
  rotors_gazebo_wind_plugin
  gazebo_sonar_plugin
  gazebo_gps_plugin
  )

# #If BUILD_ROS_INTERFACE set to ON, build gazebo_hil_interface and gazebo_motor_failure_plugin
# if (BUILD_ROS_INTERFACE)
#   add_library(gazebo_motor_failure_plugin SHARED src/gazebo_motor_failure_plugin.cpp)
#   target_link_libraries(gazebo_motor_failure_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})
#   list(APPEND plugins gazebo_motor_failure_plugin)
#   message(STATUS "adding gazebo_motor_failure_plugin to build")

#   include_directories(
#     include
#     ${mavros_msgs_INCLUDE_DIRS}
#     ${geometry_msgs_INCLUDE_DIRS}
#     ${sensor_msgs_INCLUDE_DIRS}
#     ${mav_msgs_INCLUDE_DIRS}
#     )
# endif()


# Linux is not consistent with plugin availability, even on Gazebo 7
#if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
  add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
  list(APPEND plugins LiftDragPlugin)
#endif()

foreach(plugin ${plugins})
  add_dependencies(${plugin} mav_msgs)
endforeach()

#==================================== CF ROS INTERFACE ========================================//
add_library(crazyflie_ros SHARED src/crazyflie_ros/CrazyflieROS.cpp)
target_link_libraries(crazyflie_ros ${CRAZYFLIE_COMM_LIBRARY})
add_dependencies(crazyflie_ros crazyflie_gazebo_generate_messages_cpp)
list(APPEND targets_to_install crazyflie_ros)
#==============================================================================================//

#======================================= cfGHost PLUGIN ==========================================//
add_library(gazebo_cfGhost_plugin SHARED src/gazebo_cfGhost_plugin.cpp)
# target_link_libraries(gazebo_cfGhost_plugin)
add_dependencies(gazebo_cfGhost_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_cfGhost_plugin)
#==============================================================================================//

#======================================= cfHandler PLUGIN ==========================================//
add_library(gazebo_cfHandler_plugin SHARED src/gazebo_cfHandler_plugin.cpp)
target_link_libraries(gazebo_cfHandler_plugin crazyflie_ros ${CRAZYFLIE_COMM_LIBRARY})
add_dependencies(gazebo_cfHandler_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_cfHandler_plugin)
#==============================================================================================//

#===================================== Joy Control Node =======================================//
add_executable(joy_control_node src/crazyflie_ros/joy_control.cpp)
add_dependencies(joy_control_node crazyflie_gazebo_generate_messages_cpp)
target_link_libraries(joy_control_node ${catkin_LIBRARIES})
#==============================================================================================// 

# # Configure the setup script
# if (catkin_FOUND)
#   catkin_add_env_hooks(50_sitl_gazebo_setup
#     DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
#     SHELLS sh)
# endif()

#############
## Install ##
#############

set(PLUGIN_PATH ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/plugins)
set(MODEL_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/models)
set(RESOURCE_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/models/.DS_Store)
file(GLOB models_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/models/*)

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/worlds/.DS_Store)
file(GLOB worlds_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/worlds/*)

install(TARGETS ${plugins} mav_msgs DESTINATION ${PLUGIN_PATH})
install(DIRECTORY ${models_list} DESTINATION ${MODEL_PATH})
install(FILES ${worlds_list} DESTINATION ${RESOURCE_PATH}/worlds)

configure_file(src/setup.sh.in "${CMAKE_CURRENT_BINARY_DIR}/setup.sh" @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${RESOURCE_PATH})




#############
## Testing ##
#############

###############
## Packaging ##
###############

set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT pxusers@googlegroups.com)
set(DEBIAN_PACKAGE_DEPENDS "")
set(RPM_PACKAGE_DEPENDS "")

set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEBIAN_PACKAGE_DEPENDS})
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "gazebo plugins for cf sitl.")

set(CPACK_RPM_PACKAGE_REQUIRES "${DEBIAN_PACKAGE_DEPENDS}")
set(CPACK_RPM_PACKAGE_DESCRIPTION "Gazebo plugins for cf sitl.")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")

include(CPack)
