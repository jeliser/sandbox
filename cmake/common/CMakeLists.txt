# Set the project name
project(common)

# Define the executables
add_executable(run_object application/application.cpp)
target_link_libraries(run_object dl)

