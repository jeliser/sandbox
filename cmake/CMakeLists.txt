# Check the CMake version
cmake_minimum_required(VERSION 3.1.0)

# Declare the project name
project(cpp-cmake-examples VERSION 0.0.0.0 LANGUAGES C;CXX)

# Set the output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Create the output directories
add_custom_target(build-time-make-directory ALL 
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/lib
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin) 

# Add the common directory to the build system
add_subdirectory(common)

# Add the applications directory to the build system
add_subdirectory(applications)

# Add the test directory to the build system
add_subdirectory(test)

