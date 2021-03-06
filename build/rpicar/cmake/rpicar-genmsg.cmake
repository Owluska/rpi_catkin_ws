# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rpicar: 1 messages, 1 services")

set(MSG_I_FLAGS "-Irpicar:/home/pi/catkin_ws/src/rpicar/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rpicar_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/srv/camera.srv" NAME_WE)
add_custom_target(_rpicar_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rpicar" "/home/pi/catkin_ws/src/rpicar/srv/camera.srv" "sensor_msgs/CameraInfo:sensor_msgs/RegionOfInterest:std_msgs/Header"
)

get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg" NAME_WE)
add_custom_target(_rpicar_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rpicar" "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg" "nav_msgs/Odometry:geometry_msgs/Pose:sensor_msgs/MagneticField:sensor_msgs/Temperature:geometry_msgs/TwistWithCovariance:geometry_msgs/Vector3:sensor_msgs/Range:geometry_msgs/PoseWithCovarianceStamped:geometry_msgs/PoseWithCovariance:std_msgs/Header:geometry_msgs/Twist:sensor_msgs/Imu:sensor_msgs/BatteryState:geometry_msgs/Point:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rpicar
  "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/MagneticField.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Temperature.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/BatteryState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rpicar
)

### Generating Services
_generate_srv_cpp(rpicar
  "/home/pi/catkin_ws/src/rpicar/srv/camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rpicar
)

### Generating Module File
_generate_module_cpp(rpicar
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rpicar
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rpicar_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rpicar_generate_messages rpicar_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/srv/camera.srv" NAME_WE)
add_dependencies(rpicar_generate_messages_cpp _rpicar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg" NAME_WE)
add_dependencies(rpicar_generate_messages_cpp _rpicar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rpicar_gencpp)
add_dependencies(rpicar_gencpp rpicar_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rpicar_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rpicar
  "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/MagneticField.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Temperature.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/BatteryState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rpicar
)

### Generating Services
_generate_srv_eus(rpicar
  "/home/pi/catkin_ws/src/rpicar/srv/camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rpicar
)

### Generating Module File
_generate_module_eus(rpicar
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rpicar
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rpicar_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rpicar_generate_messages rpicar_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/srv/camera.srv" NAME_WE)
add_dependencies(rpicar_generate_messages_eus _rpicar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg" NAME_WE)
add_dependencies(rpicar_generate_messages_eus _rpicar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rpicar_geneus)
add_dependencies(rpicar_geneus rpicar_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rpicar_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rpicar
  "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/MagneticField.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Temperature.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/BatteryState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rpicar
)

### Generating Services
_generate_srv_lisp(rpicar
  "/home/pi/catkin_ws/src/rpicar/srv/camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rpicar
)

### Generating Module File
_generate_module_lisp(rpicar
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rpicar
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rpicar_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rpicar_generate_messages rpicar_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/srv/camera.srv" NAME_WE)
add_dependencies(rpicar_generate_messages_lisp _rpicar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg" NAME_WE)
add_dependencies(rpicar_generate_messages_lisp _rpicar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rpicar_genlisp)
add_dependencies(rpicar_genlisp rpicar_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rpicar_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rpicar
  "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/MagneticField.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Temperature.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/BatteryState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rpicar
)

### Generating Services
_generate_srv_nodejs(rpicar
  "/home/pi/catkin_ws/src/rpicar/srv/camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rpicar
)

### Generating Module File
_generate_module_nodejs(rpicar
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rpicar
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rpicar_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rpicar_generate_messages rpicar_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/srv/camera.srv" NAME_WE)
add_dependencies(rpicar_generate_messages_nodejs _rpicar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg" NAME_WE)
add_dependencies(rpicar_generate_messages_nodejs _rpicar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rpicar_gennodejs)
add_dependencies(rpicar_gennodejs rpicar_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rpicar_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rpicar
  "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/MagneticField.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Temperature.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/BatteryState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rpicar
)

### Generating Services
_generate_srv_py(rpicar
  "/home/pi/catkin_ws/src/rpicar/srv/camera.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rpicar
)

### Generating Module File
_generate_module_py(rpicar
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rpicar
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rpicar_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rpicar_generate_messages rpicar_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/srv/camera.srv" NAME_WE)
add_dependencies(rpicar_generate_messages_py _rpicar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/rpicar/msg/telemetry.msg" NAME_WE)
add_dependencies(rpicar_generate_messages_py _rpicar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rpicar_genpy)
add_dependencies(rpicar_genpy rpicar_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rpicar_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rpicar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rpicar
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rpicar_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(rpicar_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(rpicar_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rpicar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rpicar
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rpicar_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(rpicar_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(rpicar_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rpicar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rpicar
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rpicar_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(rpicar_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(rpicar_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rpicar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rpicar
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rpicar_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(rpicar_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(rpicar_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rpicar)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rpicar\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rpicar
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rpicar_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(rpicar_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(rpicar_generate_messages_py nav_msgs_generate_messages_py)
endif()
