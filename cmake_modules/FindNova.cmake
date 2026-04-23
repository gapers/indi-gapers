# - Try to find libnova
# Once done this will define
#
#  NOVA_FOUND - system has libnova
#  NOVA_INCLUDE_DIR - the libnova include directory
#  NOVA_LIBRARIES - Link these to use libnova

find_path(NOVA_INCLUDE_DIR
  NAMES libnova/libnova.h
  HINTS
    ${_obIncDir}
    ${GNUWIN32_DIR}/include
)

find_library(NOVA_LIBRARIES
  NAMES nova libnova libnovad
  HINTS
    ${_obLinkDir}
    ${GNUWIN32_DIR}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Nova
  REQUIRED_VARS
    NOVA_INCLUDE_DIR
    NOVA_LIBRARIES
  FAIL_MESSAGE "libnova not found. Please install libnova development package."
)

if(Nova_FOUND AND NOT TARGET Nova::Nova)
  add_library(Nova::Nova UNKNOWN IMPORTED)
  set_target_properties(Nova::Nova PROPERTIES
    IMPORTED_LOCATION "${NOVA_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${NOVA_INCLUDE_DIR}"
  )
endif()

# Keep legacy variable spelling for old consumers.
set(NOVA_FOUND ${Nova_FOUND})

mark_as_advanced(NOVA_INCLUDE_DIR NOVA_LIBRARIES)
