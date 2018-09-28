# Include the GTEST CMake configuration file.  I didn't want to pollute the main file with GTEST stuff
include(CMakeLists-gtest.txt)

# Add all of the unit test files into a single unit test application
file(GLOB_RECURSE UNIT_TEST_SOURCE_FILES *.cpp)

# Create the unit test executable
add_executable(run_tests ${UNIT_TEST_SOURCE_FILES})
# Add the unit test libraries
target_link_libraries(run_tests libgtest)
