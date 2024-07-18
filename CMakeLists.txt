cmake_minimum_required (VERSION 2.8)
project (indi_gapers CXX C)

include(GNUInstallDirs)

# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# find our required packages
find_package(INDI REQUIRED)
find_package(Nova REQUIRED)
find_package(ZLIB REQUIRED)
find_package(GSL REQUIRED)

# these will be used to set the version number in config.h and our driver's xml file
set(CDRIVER_VERSION_MAJOR 2)
set(CDRIVER_VERSION_MINOR 0)

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

# do the replacement in the driver's xml file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_gapers.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_gapers.xml
)


include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${NOVA_INCLUDE_DIR})
include_directories( ${EV_INCLUDE_DIR})

include(CMakeCommon)

# Target executable
add_executable(indi_gapers driver/indi-gapers.cpp)
target_link_libraries(indi_gapers ${INDI_DRIVER_LIBRARIES} ${NOVA_LIBRARIES} ${GSL_LIBRARIES})

# tell cmake where to install our executable
install (TARGETS indi_gapers RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_gapers.xml
    DESTINATION ${INDI_DATA_DIR}
)
