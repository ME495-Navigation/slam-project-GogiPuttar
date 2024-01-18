#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "turtlelib::turtlelib" for configuration ""
set_property(TARGET turtlelib::turtlelib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(turtlelib::turtlelib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libturtlelib.a"
  )

list(APPEND _cmake_import_check_targets turtlelib::turtlelib )
list(APPEND _cmake_import_check_files_for_turtlelib::turtlelib "${_IMPORT_PREFIX}/lib/libturtlelib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
