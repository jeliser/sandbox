# Check the CMake version
cmake_minimum_required(VERSION 3.1.0)
# Declare the project name (THIS NEEDS TO HAPPEN BEFORE ANY OTHER PROCESSING)
project(cpp-cmake-examples VERSION 0.0.0.0 LANGUAGES C;CXX)

# Command line option to disable the unit tests
option(ENABLE_TESTS "Enable unit tests" OFF) 

# Validate the matched compilers
if(${CMAKE_SYSTEM_NAME} MATCHES Linux)
  # C compiler
  set(REQUIRED_C_COMPILER_VERSION 7.3.0)
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS REQUIRED_C_COMPILER_VERSION)
    message(FATAL_ERROR "Insufficient ${CMAKE_C_COMPILER} version: ${CMAKE_C_COMPILER_VERSION} < ${REQUIRED_C_COMPILER_VERSION}")
  endif()
  # C++ compiler
  set(REQUIRED_CXX_COMPILER_VERSION 7.3.0)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS REQUIRED_CXX_COMPILER_VERSION)
    message(FATAL_ERROR "Insufficient ${CMAKE_CXX_COMPILER} version: ${CMAKE_CXX_COMPILER_VERSION} < ${REQUIRED_CXX_COMPILER_VERSION}")
  endif()
else()
  message(FATAL_ERROR "Unsupported system type: ${CMAKE_SYSTEM_NAME}")
endif()  

########
## This section is probably depricated since CMAKE makes it EXTREMELY difficult to programmatically
## generate an out-of-source build.  Everything I've found says it's impossible ... why, why!

# Set the default build type if nothing is supplied
#if(NOT CMAKE_BUILD_TYPE) 
#  set(CMAKE_BUILD_TYPE "Debug")
#endif()

# Generate the output directory based on the build architecture and processor
#set(GENERATED_BUILD_DIR "build-${CMAKE_BUILD_TYPE}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
#string(TOLOWER ${GENERATED_BUILD_DIR} GENERATED_BUILD_DIR)
#set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${GENERATED_BUILD_DIR}")
##########

# Create the output directories
add_custom_target(build-time-make-directory ALL 
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/lib
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin) 

# Set the output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set the compiler defaults
set(CMAKE_CXX_STANDARD 17)

# Add the common directory to the build system
add_subdirectory(common)

# Add the applications directory to the build system
add_subdirectory(applications)

# Check the unit test command line option
if(${ENABLE_TESTS})
  # Add the test directory to the build system
  add_subdirectory(test)
endif()

