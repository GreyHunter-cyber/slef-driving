# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "plan_msgs: 8 messages, 0 services")

set(MSG_I_FLAGS "-Iplan_msgs:/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(plan_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg" ""
)

get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg" ""
)

get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg" ""
)

get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg" "plan_msgs/PointSYK"
)

get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg" ""
)

get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg" ""
)

get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg" ""
)

get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg" "plan_msgs/PointTraj"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)
_generate_msg_cpp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)
_generate_msg_cpp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)
_generate_msg_cpp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)
_generate_msg_cpp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)
_generate_msg_cpp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)
_generate_msg_cpp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)
_generate_msg_cpp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(plan_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_gencpp)
add_dependencies(plan_msgs_gencpp plan_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)
_generate_msg_eus(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)
_generate_msg_eus(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)
_generate_msg_eus(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)
_generate_msg_eus(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)
_generate_msg_eus(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)
_generate_msg_eus(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)
_generate_msg_eus(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(plan_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_geneus)
add_dependencies(plan_msgs_geneus plan_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)
_generate_msg_lisp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)
_generate_msg_lisp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)
_generate_msg_lisp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)
_generate_msg_lisp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)
_generate_msg_lisp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)
_generate_msg_lisp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)
_generate_msg_lisp(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(plan_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_genlisp)
add_dependencies(plan_msgs_genlisp plan_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)
_generate_msg_nodejs(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)
_generate_msg_nodejs(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)
_generate_msg_nodejs(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)
_generate_msg_nodejs(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)
_generate_msg_nodejs(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)
_generate_msg_nodejs(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)
_generate_msg_nodejs(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(plan_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_gennodejs)
add_dependencies(plan_msgs_gennodejs plan_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)
_generate_msg_py(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)
_generate_msg_py(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)
_generate_msg_py(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)
_generate_msg_py(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)
_generate_msg_py(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)
_generate_msg_py(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)
_generate_msg_py(plan_msgs
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(plan_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/HmiControl.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Grid.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointXY.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Path.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointSYK.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/PointTraj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/iau_ros_legoloam/src/robot_plan/plan_msgs/msg/Traj.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_genpy)
add_dependencies(plan_msgs_genpy plan_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(plan_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(plan_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(plan_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(plan_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(plan_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
