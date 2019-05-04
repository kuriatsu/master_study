# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "swipe_obstacles: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iswipe_obstacles:/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg;-Iautoware_msgs:/home/kuriatsu/Autoware/ros/src/msgs/autoware_msgs/msg;-Ijsk_recognition_msgs:/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg;-Ipcl_msgs:/opt/ros/kinetic/share/pcl_msgs/cmake/../msg;-Ivisualization_msgs:/opt/ros/kinetic/share/visualization_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Ijsk_footstep_msgs:/opt/ros/kinetic/share/jsk_footstep_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(swipe_obstacles_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg" NAME_WE)
add_custom_target(_swipe_obstacles_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "swipe_obstacles" "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg" "geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg" NAME_WE)
add_custom_target(_swipe_obstacles_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "swipe_obstacles" "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg" "geometry_msgs/Quaternion:swipe_obstacles/detected_obstacle:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swipe_obstacles
)
_generate_msg_cpp(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swipe_obstacles
)

### Generating Services

### Generating Module File
_generate_module_cpp(swipe_obstacles
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swipe_obstacles
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(swipe_obstacles_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(swipe_obstacles_generate_messages swipe_obstacles_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_cpp _swipe_obstacles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_cpp _swipe_obstacles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swipe_obstacles_gencpp)
add_dependencies(swipe_obstacles_gencpp swipe_obstacles_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swipe_obstacles_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swipe_obstacles
)
_generate_msg_eus(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swipe_obstacles
)

### Generating Services

### Generating Module File
_generate_module_eus(swipe_obstacles
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swipe_obstacles
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(swipe_obstacles_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(swipe_obstacles_generate_messages swipe_obstacles_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_eus _swipe_obstacles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_eus _swipe_obstacles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swipe_obstacles_geneus)
add_dependencies(swipe_obstacles_geneus swipe_obstacles_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swipe_obstacles_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swipe_obstacles
)
_generate_msg_lisp(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swipe_obstacles
)

### Generating Services

### Generating Module File
_generate_module_lisp(swipe_obstacles
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swipe_obstacles
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(swipe_obstacles_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(swipe_obstacles_generate_messages swipe_obstacles_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_lisp _swipe_obstacles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_lisp _swipe_obstacles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swipe_obstacles_genlisp)
add_dependencies(swipe_obstacles_genlisp swipe_obstacles_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swipe_obstacles_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swipe_obstacles
)
_generate_msg_nodejs(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swipe_obstacles
)

### Generating Services

### Generating Module File
_generate_module_nodejs(swipe_obstacles
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swipe_obstacles
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(swipe_obstacles_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(swipe_obstacles_generate_messages swipe_obstacles_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_nodejs _swipe_obstacles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_nodejs _swipe_obstacles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swipe_obstacles_gennodejs)
add_dependencies(swipe_obstacles_gennodejs swipe_obstacles_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swipe_obstacles_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swipe_obstacles
)
_generate_msg_py(swipe_obstacles
  "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swipe_obstacles
)

### Generating Services

### Generating Module File
_generate_module_py(swipe_obstacles
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swipe_obstacles
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(swipe_obstacles_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(swipe_obstacles_generate_messages swipe_obstacles_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_py _swipe_obstacles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg/detected_obstacle_array.msg" NAME_WE)
add_dependencies(swipe_obstacles_generate_messages_py _swipe_obstacles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swipe_obstacles_genpy)
add_dependencies(swipe_obstacles_genpy swipe_obstacles_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swipe_obstacles_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swipe_obstacles)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swipe_obstacles
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET autoware_msgs_generate_messages_cpp)
  add_dependencies(swipe_obstacles_generate_messages_cpp autoware_msgs_generate_messages_cpp)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_cpp)
  add_dependencies(swipe_obstacles_generate_messages_cpp jsk_recognition_msgs_generate_messages_cpp)
endif()
if(TARGET pcl_msgs_generate_messages_cpp)
  add_dependencies(swipe_obstacles_generate_messages_cpp pcl_msgs_generate_messages_cpp)
endif()
if(TARGET visualization_msgs_generate_messages_cpp)
  add_dependencies(swipe_obstacles_generate_messages_cpp visualization_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(swipe_obstacles_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swipe_obstacles)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swipe_obstacles
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET autoware_msgs_generate_messages_eus)
  add_dependencies(swipe_obstacles_generate_messages_eus autoware_msgs_generate_messages_eus)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_eus)
  add_dependencies(swipe_obstacles_generate_messages_eus jsk_recognition_msgs_generate_messages_eus)
endif()
if(TARGET pcl_msgs_generate_messages_eus)
  add_dependencies(swipe_obstacles_generate_messages_eus pcl_msgs_generate_messages_eus)
endif()
if(TARGET visualization_msgs_generate_messages_eus)
  add_dependencies(swipe_obstacles_generate_messages_eus visualization_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(swipe_obstacles_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swipe_obstacles)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swipe_obstacles
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET autoware_msgs_generate_messages_lisp)
  add_dependencies(swipe_obstacles_generate_messages_lisp autoware_msgs_generate_messages_lisp)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_lisp)
  add_dependencies(swipe_obstacles_generate_messages_lisp jsk_recognition_msgs_generate_messages_lisp)
endif()
if(TARGET pcl_msgs_generate_messages_lisp)
  add_dependencies(swipe_obstacles_generate_messages_lisp pcl_msgs_generate_messages_lisp)
endif()
if(TARGET visualization_msgs_generate_messages_lisp)
  add_dependencies(swipe_obstacles_generate_messages_lisp visualization_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(swipe_obstacles_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swipe_obstacles)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swipe_obstacles
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET autoware_msgs_generate_messages_nodejs)
  add_dependencies(swipe_obstacles_generate_messages_nodejs autoware_msgs_generate_messages_nodejs)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_nodejs)
  add_dependencies(swipe_obstacles_generate_messages_nodejs jsk_recognition_msgs_generate_messages_nodejs)
endif()
if(TARGET pcl_msgs_generate_messages_nodejs)
  add_dependencies(swipe_obstacles_generate_messages_nodejs pcl_msgs_generate_messages_nodejs)
endif()
if(TARGET visualization_msgs_generate_messages_nodejs)
  add_dependencies(swipe_obstacles_generate_messages_nodejs visualization_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(swipe_obstacles_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swipe_obstacles)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swipe_obstacles\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swipe_obstacles
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET autoware_msgs_generate_messages_py)
  add_dependencies(swipe_obstacles_generate_messages_py autoware_msgs_generate_messages_py)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_py)
  add_dependencies(swipe_obstacles_generate_messages_py jsk_recognition_msgs_generate_messages_py)
endif()
if(TARGET pcl_msgs_generate_messages_py)
  add_dependencies(swipe_obstacles_generate_messages_py pcl_msgs_generate_messages_py)
endif()
if(TARGET visualization_msgs_generate_messages_py)
  add_dependencies(swipe_obstacles_generate_messages_py visualization_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(swipe_obstacles_generate_messages_py std_msgs_generate_messages_py)
endif()
