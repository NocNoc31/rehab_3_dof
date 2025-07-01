#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "odrive_can_interface::odrive_can_interface" for configuration ""
set_property(TARGET odrive_can_interface::odrive_can_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(odrive_can_interface::odrive_can_interface PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libodrive_can_interface.so"
  IMPORTED_SONAME_NOCONFIG "libodrive_can_interface.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS odrive_can_interface::odrive_can_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_odrive_can_interface::odrive_can_interface "${_IMPORT_PREFIX}/lib/libodrive_can_interface.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
