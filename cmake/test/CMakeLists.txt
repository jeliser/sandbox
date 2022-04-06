# Include the GTEST CMake configuration file.  I didn't want to pollute the main file with GTEST stuff
include(.CMakeLists-gtest.txt)

# Add all of the unit test files into a single unit test application
file(GLOB_RECURSE UNIT_TEST_SOURCE_FILES *.cpp)

# Create the unit test executable
set(UNIT_TEST run_tests)
add_executable(${UNIT_TEST} ${UNIT_TEST_SOURCE_FILES})
# Add the unit test libraries
add_dependencies(${UNIT_TEST} gtest)
target_link_libraries(${UNIT_TEST} "${binary_dir}/lib/libgtest.a" pthread)

# code coverage target
string(APPEND CMAKE_C_FLAGS "--coverage -fprofile-arcs -ftest-coverage ")
string(APPEND CMAKE_CXX_FLAGS "--coverage -fprofile-arcs -ftest-coverage ")

add_custom_target(run-tests
  COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${UNIT_TEST}
  DEPENDS ${UNIT_TEST})

add_custom_target(valgrind
  COMMAND valgrind --tool=memcheck --leak-check=yes ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${UNIT_TEST} &> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${UNIT_TEST}.val
  DEPENDS ${UNIT_TEST})

set(GCOV_EXCLUDE "\\\"/usr/*\\\" \\\"*/gtest*\\\"")
add_custom_target(coverage
  COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/run_tests --gtest_filter=AnotherExample.SampleTest_PASS
  COMMAND lcov --rc lcov_branch_coverage=0 --gcov-tool gcov --capture --directory . --output-file coverage.info
  ## Some reason the GCOV_EXCLUDE when included the COMMAND just doesn't escape the quotes correctly :(
  COMMAND bash -c "lcov --rc lcov_branch_coverage=0 --gcov-tool gcov --remove coverage.info ${GCOV_EXCLUDE} -o coverage.info"
  COMMAND genhtml --prefix ${PROJECT_NAME} --no-branch-coverage coverage.info --output-directory ../cov
  DEPENDS ${UNIT_TEST})


