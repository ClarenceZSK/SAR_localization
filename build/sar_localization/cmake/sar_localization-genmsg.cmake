# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sar_localization: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isar_localization:/home/clarence/SAR_Localization/src/sar_localization/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sar_localization_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sar_localization
  "/home/clarence/SAR_Localization/src/sar_localization/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization
)

### Generating Services

### Generating Module File
_generate_module_cpp(sar_localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sar_localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sar_localization_generate_messages sar_localization_generate_messages_cpp)

# target for backward compatibility
add_custom_target(sar_localization_gencpp)
add_dependencies(sar_localization_gencpp sar_localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sar_localization_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sar_localization
  "/home/clarence/SAR_Localization/src/sar_localization/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization
)

### Generating Services

### Generating Module File
_generate_module_lisp(sar_localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sar_localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sar_localization_generate_messages sar_localization_generate_messages_lisp)

# target for backward compatibility
add_custom_target(sar_localization_genlisp)
add_dependencies(sar_localization_genlisp sar_localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sar_localization_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sar_localization
  "/home/clarence/SAR_Localization/src/sar_localization/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization
)

### Generating Services

### Generating Module File
_generate_module_py(sar_localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sar_localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sar_localization_generate_messages sar_localization_generate_messages_py)

# target for backward compatibility
add_custom_target(sar_localization_genpy)
add_dependencies(sar_localization_genpy sar_localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sar_localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(sar_localization_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(sar_localization_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(sar_localization_generate_messages_py std_msgs_generate_messages_py)
