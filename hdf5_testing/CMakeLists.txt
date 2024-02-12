cmake_minimum_required(VERSION 3.10)
project(MyHDF5Project)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find the HDF5 package
find_package(HDF5 REQUIRED COMPONENTS CXX)

# Add the HDF5 include directories
include_directories(${HDF5_INCLUDE_DIRS})

# Add the executable
add_executable(hdf5_testing testing.cpp)
target_link_libraries(hdf5_testing ${HDF5_LIBRARIES})

add_executable(hdf5_appending testing_append.cpp)
target_link_libraries(hdf5_appending ${HDF5_LIBRARIES})
