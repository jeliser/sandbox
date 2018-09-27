# Check the CMake version
cmake_minimum_required(VERSION 3.0.0)

# Declare the project name
project(cpp-examples VERSION 0.0.0.0 LANGUAGES C;CXX)

# Set the output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
Set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Define the executables
add_executable(run_object common/application/application.cpp)
target_link_libraries(run_object dl)

# Define the shared objects
include_directories(./)
add_library(example SHARED applications/example/shared_object.cpp)

