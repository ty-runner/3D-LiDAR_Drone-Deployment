#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "servo_control::servo_control__rosidl_typesupport_introspection_cpp" for configuration ""
set_property(TARGET servo_control::servo_control__rosidl_typesupport_introspection_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(servo_control::servo_control__rosidl_typesupport_introspection_cpp PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libservo_control__rosidl_typesupport_introspection_cpp.so"
  IMPORTED_SONAME_NOCONFIG "libservo_control__rosidl_typesupport_introspection_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS servo_control::servo_control__rosidl_typesupport_introspection_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_servo_control::servo_control__rosidl_typesupport_introspection_cpp "${_IMPORT_PREFIX}/lib/libservo_control__rosidl_typesupport_introspection_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
