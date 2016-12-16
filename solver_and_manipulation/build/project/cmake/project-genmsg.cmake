# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "project: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iproject:/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(project_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg/solution.msg" NAME_WE)
add_custom_target(_project_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "project" "/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg/solution.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(project
  "/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg/solution.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/project
)

### Generating Services

### Generating Module File
_generate_module_cpp(project
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/project
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(project_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(project_generate_messages project_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg/solution.msg" NAME_WE)
add_dependencies(project_generate_messages_cpp _project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(project_gencpp)
add_dependencies(project_gencpp project_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS project_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(project
  "/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg/solution.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/project
)

### Generating Services

### Generating Module File
_generate_module_lisp(project
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/project
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(project_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(project_generate_messages project_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg/solution.msg" NAME_WE)
add_dependencies(project_generate_messages_lisp _project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(project_genlisp)
add_dependencies(project_genlisp project_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS project_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(project
  "/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg/solution.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project
)

### Generating Services

### Generating Module File
_generate_module_py(project
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(project_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(project_generate_messages project_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg/solution.msg" NAME_WE)
add_dependencies(project_generate_messages_py _project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(project_genpy)
add_dependencies(project_genpy project_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS project_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/project
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(project_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/project
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(project_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(project_generate_messages_py std_msgs_generate_messages_py)
