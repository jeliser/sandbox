# Include the GTEST CMake configuration file.  I didn't want to pollute the main file with GTEST stuff
include(${CMAKE_SOURCE_DIR}/.cmake/gtest.cmake)

# Add in the example libraries (this list can be generated if needed)
set(EXAMPLE_LIBS example another_example)

# Add all of the unit test files into a single unit test application
file(GLOB_RECURSE UNIT_TEST_SOURCE_FILES *.cpp)

# A crude method to filter down the test applications
#string(REPLACE "*" "" TEST_EXE_PATTERN "${TEST_EXE}")

# Create the unit test executable
set(UNIT_TEST run_tests)
add_executable(${UNIT_TEST} ${UNIT_TEST_SOURCE_FILES})
# Add the unit test libraries
add_dependencies(${UNIT_TEST} ${EXAMPLE_LIBS} prebuild gtest)
#target_link_libraries(${UNIT_TEST} "${binary_dir}/lib/libgtest.a" "${binary_dir}/lib/libgmock.a" pthread)  # URL gtest-master
target_link_libraries(${UNIT_TEST} libgtest libgmock dl pthread) # URL gtest-1.8.0
target_include_directories(${UNIT_TEST} PRIVATE ${PROJECT_INCLUDE} ${CMAKE_SOURCE_DIR})

# code coverage target
string(APPEND CMAKE_C_FLAGS "--coverage -fprofile-arcs -ftest-coverage ")
string(APPEND CMAKE_CXX_FLAGS "--coverage -fprofile-arcs -ftest-coverage ")

set(UNIT_TEST_XML_PATH "./test_xml")
add_custom_target(run-tests
  COMMAND find ${CMAKE_BINARY_DIR} -name *gcd* | xargs rm -f
  COMMAND bash -c "ASAN_OPTIONS=halt_on_error=0 LSAN_OPTIONS=suppressions=${CMAKE_SOURCE_DIR}/.leak.txt ./${UNIT_TEST} --gtest_output=xml:${UNIT_TEST_XML_PATH}/${UNIT_TEST}.xml ${TEST_ARGS}" | tee output.log
  # If you happen to have a bunch of standalone unit tests, this will help know which ones failed
  #COMMAND cat output.log | grep "  FAILED  " | wc -l >> /dev/null && printf \"\\n\\n--------------------\\n[   ALL FAILED TESTS   ]\\n\\n\" && cat output.log | grep "  FAILED  " | grep -v " listed below" | grep -v " ms" && printf \"\\n\"
  WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  DEPENDS ${UNIT_TEST})

set(GCOV_EXCLUDE "\\\"/usr/*\\\" \\\"*/gtest*\\\" \\\"*/opt*\\\"")
add_custom_target(coverage
  COMMAND lcov --rc lcov_branch_coverage=1 --gcov-tool gcov --capture --directory . --output-file unfiltered.info
  ## Some reason the GCOV_EXCLUDE when included the COMMAND just doesn't escape the quotes correctly :(
  COMMAND bash -c "lcov --rc lcov_branch_coverage=1 --gcov-tool gcov --remove unfiltered.info ${GCOV_EXCLUDE} -o coverage.info"
  COMMAND genhtml --demangle-cpp --prefix ${PROJECT_NAME} --branch-coverage coverage.info --output-directory cov
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS run-tests)

add_custom_target(valgrind
  COMMAND valgrind --tool=memcheck --leak-check=yes ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${UNIT_TEST} &> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${UNIT_TEST}.val
  DEPENDS ${UNIT_TEST})

