# Set the project name
project(another_example_shared_object)

# The shared object name will be the current working directory basename
#   We're passing in a path and extracting the name (so it's not really a filename)
get_filename_component(LIB_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

# Include the top level path for #include resolution
include_directories(${CMAKE_SOURCE_DIR})

# Create the shared object using the generated directory name and the source file explicitly
add_library(${LIB_NAME} SHARED shared_object.cpp)

