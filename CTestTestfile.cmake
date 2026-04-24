# CMake generated Testfile for 
# Source directory: /home/negro/dev/gapers/indi-gapers
# Build directory: /home/negro/dev/gapers/indi-gapers
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(indi_gapers_motion_calculations "python3" "/home/negro/dev/gapers/indi-gapers/scripts/test_motion_and_plc_commands.py")
set_tests_properties(indi_gapers_motion_calculations PROPERTIES  WORKING_DIRECTORY "/home/negro/dev/gapers/indi-gapers" _BACKTRACE_TRIPLES "/home/negro/dev/gapers/indi-gapers/CMakeLists.txt;90;add_test;/home/negro/dev/gapers/indi-gapers/CMakeLists.txt;0;")
