# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "manipulation: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imanipulation:/home/team4/ros_workspace/project/src/manipulation/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(manipulation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/team4/ros_workspace/project/src/manipulation/msg/solution.msg" NAME_WE)
add_custom_target(_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation" "/home/team4/ros_workspace/project/src/manipulation/msg/solution.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(manipulation
  "/home/team4/ros_workspace/project/src/manipulation/msg/solution.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation
)

### Generating Services

### Generating Module File
_generate_module_cpp(manipulation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(manipulation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(manipulation_generate_messages manipulation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/team4/ros_workspace/project/src/manipulation/msg/solution.msg" NAME_WE)
add_dependencies(manipulation_generate_messages_cpp _manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manipulation_gencpp)
add_dependencies(manipulation_gencpp manipulation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manipulation_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(manipulation
  "/home/team4/ros_workspace/project/src/manipulation/msg/solution.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manipulation
)

### Generating Services

### Generating Module File
_generate_module_lisp(manipulation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manipulation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(manipulation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(manipulation_generate_messages manipulation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/team4/ros_workspace/project/src/manipulation/msg/solution.msg" NAME_WE)
add_dependencies(manipulation_generate_messages_lisp _manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manipulation_genlisp)
add_dependencies(manipulation_genlisp manipulation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manipulation_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(manipulation
  "/home/team4/ros_workspace/project/src/manipulation/msg/solution.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation
)

### Generating Services

### Generating Module File
_generate_module_py(manipulation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(manipulation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(manipulation_generate_messages manipulation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/team4/ros_workspace/project/src/manipulation/msg/solution.msg" NAME_WE)
add_dependencies(manipulation_generate_messages_py _manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manipulation_genpy)
add_dependencies(manipulation_genpy manipulation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manipulation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(manipulation_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manipulation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(manipulation_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(manipulation_generate_messages_py std_msgs_generate_messages_py)
