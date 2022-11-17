# Check the CMake version
cmake_minimum_required(VERSION 2.7.0)
# Declare the project name (THIS NEEDS TO HAPPEN BEFORE ANY OTHER PROCESSING)
if(${CMAKE_VERSION} VERSION_LESS "3.0.0")
  ## CMake < 3.0
  project(cpp-cmake-examples)
else()
  ## CMake >= 3.0  (You can't have the additional arguments)
  cmake_policy(SET CMP0048 NEW)
  cmake_policy(SET CMP0012 NEW)
  project(cpp-cmake-examples VERSION 0.0.0.0 LANGUAGES C;CXX)
endif()

# Command line option to disable the unit tests
option(TESTS "Enable unit tests" OFF) 
option(TEST_ARGS "Command line arguments to be passed to the test executor" "") 
option(DOCS "Build the documentation" OFF)
option(LIB_TYPE "The type of oject to create: SHARED, STATIC.  Default: STATIC" STATIC)

# Include some top-level common macros
#include(${CMAKE_SOURCE_DIR}/cmake/environment/common.cmake)

# Make sure the LIB_TYPE is upper case
string(TOUPPER ${LIB_TYPE} LIB_TYPE)

get_filename_component(TOOLCHAIN_FILE ${CMAKE_TOOLCHAIN_FILE} NAME)
if(EXISTS ".cmake/environment/${TOOLCHAIN_FILE}")
  include(.cmake/environment/${TOOLCHAIN_FILE})
endif()


# Validate the matched compilers
if(${CMAKE_SYSTEM_NAME} MATCHES Linux)
  # C compiler
  set(REQUIRED_C_COMPILER_VERSION 4.3.0)
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS REQUIRED_C_COMPILER_VERSION)
    message(FATAL_ERROR "Insufficient ${CMAKE_C_COMPILER} version: ${CMAKE_C_COMPILER_VERSION} < ${REQUIRED_C_COMPILER_VERSION}")
  endif()
  # C++ compiler
  set(REQUIRED_CXX_COMPILER_VERSION 4.3.0)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS REQUIRED_CXX_COMPILER_VERSION)
    message(FATAL_ERROR "Insufficient ${CMAKE_CXX_COMPILER} version: ${CMAKE_CXX_COMPILER_VERSION} < ${REQUIRED_CXX_COMPILER_VERSION}")
  endif()
else()
  message(FATAL_ERROR "Unsupported system type: ${CMAKE_SYSTEM_NAME}")
endif()  

# Set the output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Create the output directories
add_custom_target(prebuild-make-directory ALL 
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
# Create the 'latest' symlink pointer
get_filename_component(REL_BINARY_PATH ${CMAKE_BINARY_DIR} NAME)
add_custom_target(prebuild-make-latest ALL 
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/..
  COMMAND rm -f latest && ln -sf ${REL_BINARY_PATH} latest)
add_custom_target(prebuild ALL DEPENDS prebuild-make-directory prebuild-make-latest)

# Set the compiler defaults
set(CMAKE_CXX_STANDARD 14)

# Set the common flags
set(COMMON_FLAGS "${WARNINGS} -Wall ")

# Load the compiler flags
string(APPEND CMAKE_C_FLAGS "${COMMON_FLAGS} ${BUILD_SPECIFIC_FLAGS} ")
string(APPEND CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${BUILD_SPECIFIC_FLAGS} ")

# Add the common directory to the build system
add_subdirectory(common)

# Add the applications directory to the build system
add_subdirectory(applications)

# Check the unit test command line option
if(${TESTS})
  # Add the test directory to the build system
  add_subdirectory(test)
endif()

# Let's generate some documentation
if(${DOCS})
  find_package(Doxygen)
  if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM )
  else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
  endif (DOXYGEN_FOUND) 
endif()
