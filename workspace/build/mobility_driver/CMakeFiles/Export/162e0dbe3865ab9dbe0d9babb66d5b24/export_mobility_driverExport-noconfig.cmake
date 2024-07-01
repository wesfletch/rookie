#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mobility_driver::mobility_driver_lib" for configuration ""
set_property(TARGET mobility_driver::mobility_driver_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(mobility_driver::mobility_driver_lib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmobility_driver_lib.a"
  )

list(APPEND _cmake_import_check_targets mobility_driver::mobility_driver_lib )
list(APPEND _cmake_import_check_files_for_mobility_driver::mobility_driver_lib "${_IMPORT_PREFIX}/lib/libmobility_driver_lib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
