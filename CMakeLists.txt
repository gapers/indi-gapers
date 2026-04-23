cmake_minimum_required (VERSION 3.5)
cmake_policy(SET CMP0048 NEW)

# Read project version from VERSION file (single source of truth).
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" PROJECT_VERSION_STRING LIMIT_COUNT 1)
string(STRIP "${PROJECT_VERSION_STRING}" PROJECT_VERSION_STRING)
if(NOT PROJECT_VERSION_STRING MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
    message(FATAL_ERROR "VERSION must be in MAJOR.MINOR.PATCH format (e.g. 3.0.0)")
endif()

project (indi_gapers VERSION ${PROJECT_VERSION_STRING} LANGUAGES CXX C)

include(CTest)
enable_testing()

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
add_executable(indi_gapers src/indi-gapers.cpp src/gapers_math.cpp)

# Prefer imported targets when available, while keeping compatibility with
# legacy Find modules that export plain variables.
set(INDI_GAPERS_NOVA_LINK NOVA_LIBRARIES)
if(TARGET Nova::Nova)
    set(INDI_GAPERS_NOVA_LINK Nova::Nova)
endif()

set(INDI_GAPERS_GSL_LINK GSL_LIBRARIES)
if(TARGET GSL::gsl)
    set(INDI_GAPERS_GSL_LINK GSL::gsl)
endif()

target_include_directories(indi_gapers PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${INDI_INCLUDE_DIR}
)
if(NOT TARGET Nova::Nova)
    target_include_directories(indi_gapers PRIVATE ${NOVA_INCLUDE_DIR})
endif()

target_compile_features(indi_gapers PRIVATE cxx_std_17)
target_link_libraries(indi_gapers PRIVATE
    ${INDI_DRIVER_LIBRARIES}
    ${INDI_GAPERS_NOVA_LINK}
    ${INDI_GAPERS_GSL_LINK}
)

# tell cmake where to install our executable
install (TARGETS indi_gapers RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_gapers.xml
    DESTINATION ${INDI_DATA_DIR}
)

if(BUILD_TESTING)
    # ------------------------------------------------------------------
    # C++ unit tests via Google Test (tests real driver math code)
    # ------------------------------------------------------------------
    find_package(GTest QUIET)
    if(GTest_FOUND)
        add_executable(gapers_math_tests
            tests/test_gapers_math.cpp
            src/gapers_math.cpp
        )
        target_include_directories(gapers_math_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
        target_compile_features(gapers_math_tests PRIVATE cxx_std_17)
        target_link_libraries(gapers_math_tests PRIVATE GTest::GTest GTest::Main)
        add_test(NAME gapers_math_tests COMMAND gapers_math_tests)
    else()
        message(STATUS "GTest not found — skipping C++ unit tests (install libgtest-dev)")
    endif()

    # ------------------------------------------------------------------
    # Python smoke tests (kept for quick sanity checks, no extra deps)
    # ------------------------------------------------------------------
    add_test(
        NAME indi_gapers_motion_calculations
        COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_motion_and_plc_commands.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()
