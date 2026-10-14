#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "xhand_control" for configuration "Release"
set_property(TARGET xhand_control APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(xhand_control PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libxhand_control.so"
  IMPORTED_SONAME_RELEASE "libxhand_control.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS xhand_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_xhand_control "${_IMPORT_PREFIX}/lib/libxhand_control.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
