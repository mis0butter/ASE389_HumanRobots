#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "towr::towr" for configuration "Release"
set_property(TARGET towr::towr APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(towr::towr PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libtowr.so"
  IMPORTED_SONAME_RELEASE "libtowr.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS towr::towr )
list(APPEND _IMPORT_CHECK_FILES_FOR_towr::towr "${_IMPORT_PREFIX}/lib/libtowr.so" )

# Import target "towr::towr-example" for configuration "Release"
set_property(TARGET towr::towr-example APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(towr::towr-example PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/towr/towr-example"
  )

list(APPEND _IMPORT_CHECK_TARGETS towr::towr-example )
list(APPEND _IMPORT_CHECK_FILES_FOR_towr::towr-example "${_IMPORT_PREFIX}/lib/towr/towr-example" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
