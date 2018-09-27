# Build the shared object
include_directories(${PROJECT_SOURCE_DIR})
add_library(example SHARED shared_object.cpp)

