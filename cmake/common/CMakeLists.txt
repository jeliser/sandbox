# Set the project name
project(common)

# Define the executables
add_executable(load_shared_object application/application.cpp)
target_link_libraries(load_shared_object dl)

