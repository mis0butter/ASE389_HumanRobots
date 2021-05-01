# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "towr_ros: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itowr_ros:/home/june/catkin_ws/src/towr/towr_ros/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Ixpp_msgs:/home/june/catkin_ws/src/xpp/xpp_msgs/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(towr_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg" NAME_WE)
add_custom_target(_towr_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "towr_ros" "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg" "xpp_msgs/StateLin3d:geometry_msgs/Point:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(towr_ros
  "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/june/catkin_ws/src/xpp/xpp_msgs/msg/StateLin3d.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/towr_ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(towr_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/towr_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(towr_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(towr_ros_generate_messages towr_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg" NAME_WE)
add_dependencies(towr_ros_generate_messages_cpp _towr_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(towr_ros_gencpp)
add_dependencies(towr_ros_gencpp towr_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS towr_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(towr_ros
  "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/june/catkin_ws/src/xpp/xpp_msgs/msg/StateLin3d.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/towr_ros
)

### Generating Services

### Generating Module File
_generate_module_eus(towr_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/towr_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(towr_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(towr_ros_generate_messages towr_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg" NAME_WE)
add_dependencies(towr_ros_generate_messages_eus _towr_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(towr_ros_geneus)
add_dependencies(towr_ros_geneus towr_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS towr_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(towr_ros
  "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/june/catkin_ws/src/xpp/xpp_msgs/msg/StateLin3d.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/towr_ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(towr_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/towr_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(towr_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(towr_ros_generate_messages towr_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg" NAME_WE)
add_dependencies(towr_ros_generate_messages_lisp _towr_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(towr_ros_genlisp)
add_dependencies(towr_ros_genlisp towr_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS towr_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(towr_ros
  "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/june/catkin_ws/src/xpp/xpp_msgs/msg/StateLin3d.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/towr_ros
)

### Generating Services

### Generating Module File
_generate_module_nodejs(towr_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/towr_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(towr_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(towr_ros_generate_messages towr_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg" NAME_WE)
add_dependencies(towr_ros_generate_messages_nodejs _towr_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(towr_ros_gennodejs)
add_dependencies(towr_ros_gennodejs towr_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS towr_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(towr_ros
  "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/june/catkin_ws/src/xpp/xpp_msgs/msg/StateLin3d.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/towr_ros
)

### Generating Services

### Generating Module File
_generate_module_py(towr_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/towr_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(towr_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(towr_ros_generate_messages towr_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/june/catkin_ws/src/towr/towr_ros/msg/TowrCommand.msg" NAME_WE)
add_dependencies(towr_ros_generate_messages_py _towr_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(towr_ros_genpy)
add_dependencies(towr_ros_genpy towr_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS towr_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/towr_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/towr_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(towr_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET xpp_msgs_generate_messages_cpp)
  add_dependencies(towr_ros_generate_messages_cpp xpp_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/towr_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/towr_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(towr_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET xpp_msgs_generate_messages_eus)
  add_dependencies(towr_ros_generate_messages_eus xpp_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/towr_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/towr_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(towr_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET xpp_msgs_generate_messages_lisp)
  add_dependencies(towr_ros_generate_messages_lisp xpp_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/towr_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/towr_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(towr_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET xpp_msgs_generate_messages_nodejs)
  add_dependencies(towr_ros_generate_messages_nodejs xpp_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/towr_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/towr_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/towr_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(towr_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET xpp_msgs_generate_messages_py)
  add_dependencies(towr_ros_generate_messages_py xpp_msgs_generate_messages_py)
endif()
