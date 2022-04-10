# Set the project name
project(common)

# Define the executables
set(APP_NAME load_shared_object)
add_executable(${APP_NAME} application/application.cpp)
target_link_libraries(${APP_NAME} dl)

set_target_properties(${APP_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
  INSTALL_RPATH "\$ORIGIN/../lib:${INSTALL_RPATH}")

