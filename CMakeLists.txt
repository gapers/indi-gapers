cmake_minimum_required (VERSION 2.6)
project (indi_gapers CXX C)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set(BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")

# Required packages
find_package(INDI REQUIRED)
find_package(Nova REQUIRED)

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${NOVA_INCLUDE_DIR})

# Target executable
add_executable(indi_gapers driver/indi-gapers.cpp)
target_link_libraries(indi_gapers ${INDI_DRIVER_LIBRARIES})

# Installation target
install (TARGETS indi_gapers DESTINATION bin)
#install(FILES indi_aldiroof.xml DESTINATION ${INDI_DATA_DIR})
