cmake_minimum_required (VERSION 3.5)
cmake_policy(SET CMP0048 NEW)

# Read project version from VERSION file (single source of truth).
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" PROJECT_VERSION_STRING LIMIT_COUNT 1)
string(STRIP "${PROJECT_VERSION_STRING}" PROJECT_VERSION_STRING)
if(NOT PROJECT_VERSION_STRING MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
    message(FATAL_ERROR "VERSION must be in MAJOR.MINOR.PATCH format (e.g. 3.0.0)")
endif()

project (indi_gapers VERSION ${PROJECT_VERSION_STRING} LANGUAGES CXX C)

include(GNUInstallDirs)

# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# find our required packages
find_package(INDI REQUIRED)
find_package(Nova REQUIRED)
find_package(ZLIB REQUIRED)
find_package(GSL REQUIRED)

# These are used in config.h and driver's XML metadata.
set(CDRIVER_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CDRIVER_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CDRIVER_VERSION_PATCH ${PROJECT_VERSION_PATCH})

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

# do the replacement in the driver's xml file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/data/indi_gapers.xml.in
    ${CMAKE_CURRENT_BINARY_DIR}/indi_gapers.xml
)

include(CMakeCommon)

# Target executable
add_executable(indi_gapers src/indi-gapers.cpp)
target_include_directories(indi_gapers PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${INDI_INCLUDE_DIR}
    ${NOVA_INCLUDE_DIR}
)
target_compile_features(indi_gapers PRIVATE cxx_std_11)
target_link_libraries(indi_gapers ${INDI_DRIVER_LIBRARIES} ${NOVA_LIBRARIES} ${GSL_LIBRARIES})

# tell cmake where to install our executable
install (TARGETS indi_gapers RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_gapers.xml
    DESTINATION ${INDI_DATA_DIR}
)
