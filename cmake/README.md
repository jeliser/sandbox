# Install CMake

```bash
> sudo apt-get install cmake
# If you care about the GraphViz output
> sudo apt-get install graphviz
```

# Generate the Makefiles

```bash
# Just generate the Makefile 
> cmake -H. -Bbuild

# Generate the Makefile and some graphviz sweetness
> cmake -H. -Bbuild --graphviz=example
> dot -Tpng -oexample.png example
> eog example.png
```

# Run the executable

```bash
[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake]  (git:master:0e61139) 
> cd build

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build]  (git:master:0e61139) 
 > make
Scanning dependencies of target build-time-make-directory
[  0%] Built target build-time-make-directory
Scanning dependencies of target load_shared_object
[ 16%] Building CXX object common/CMakeFiles/load_shared_object.dir/application/application.cpp.o
[ 33%] Linking CXX executable ../bin/load_shared_object
[ 33%] Built target load_shared_object
Scanning dependencies of target example
[ 50%] Building CXX object applications/example/CMakeFiles/example.dir/shared_object.cpp.o
[ 66%] Linking CXX shared library ../../lib/libexample.so
[ 66%] Built target example
Scanning dependencies of target another_example
[ 83%] Building CXX object applications/another_example/CMakeFiles/another_example.dir/shared_object.cpp.o
[100%] Linking CXX shared library ../../lib/libanother_example.so
[100%] Built target another_example

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build]  (git:master:0e61139) 
> cd bin

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:e166b5b) 
 > ./load_shared_object ../lib/libexample.so
Loading shared object: ../lib/libexample.so - SUCCESS
Failed to find 'ctest1' - ../lib/libexample.so: undefined symbol: ctest1
Found 'hello_world' - 12345
Found 'newInstance' - hello world this is a list
Closing shared object: ../lib/libexample.so - SUCCESS

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:0e61139) 
 > ./load_shared_object ../lib/libanother_example.so
Loading shared object: ../lib/libanother_example.so - SUCCESS
Failed to find 'ctest1' - ../lib/libanother_example.so: undefined symbol: ctest1
Found 'hello_world' - 12345
Found 'newInstance' - hello world this is a list
Closing shared object: ../lib/libanother_example.so - SUCCESS

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:0e61139) 
 > ./load_shared_object
Usage: ./load_shared_object [shared_object_to_load]

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:0e61139) 
 > ./load_shared_object made_up_lib
Loading shared object: made_up_lib - FAILED (made_up_lib: cannot open shared object file: No such file or directory)

```

# Run the Unit Test example

## Run all the unit tests

```bash
[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:b189392) 
 > ./run_tests
[==========] Running 4 tests from 2 test cases.
[----------] Global test environment set-up.
[----------] 2 tests from AnotherExample
[ RUN      ] AnotherExample.SampleTest_PASS
[       OK ] AnotherExample.SampleTest_PASS (0 ms)
[ RUN      ] AnotherExample.SampleTest_FAIL
/home/jeliser/code/github_sandbox/cmake/test/applications/another_example/sample_unit_test.cpp:10: Failure
Value of: false
  Actual: false
Expected: true
[  FAILED  ] AnotherExample.SampleTest_FAIL (0 ms)
[----------] 2 tests from AnotherExample (0 ms total)

[----------] 2 tests from Example
[ RUN      ] Example.SampleTest_PASS
[       OK ] Example.SampleTest_PASS (0 ms)
[ RUN      ] Example.SampleTest_FAIL
/home/jeliser/code/github_sandbox/cmake/test/applications/example/sample_unit_test.cpp:10: Failure
Value of: false
  Actual: false
Expected: true
[  FAILED  ] Example.SampleTest_FAIL (0 ms)
[----------] 2 tests from Example (0 ms total)

[----------] Global test environment tear-down
[==========] 4 tests from 2 test cases ran. (0 ms total)
[  PASSED  ] 2 tests.
[  FAILED  ] 2 tests, listed below:
[  FAILED  ] AnotherExample.SampleTest_FAIL
[  FAILED  ] Example.SampleTest_FAIL

 2 FAILED TESTS
```

## Filter the unit tests that are executed

```bash
[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:b189392) 
 > ./run_tests --gtest_filter=Example*
Note: Google Test filter = Example*
[==========] Running 2 tests from 1 test case.
[----------] Global test environment set-up.
[----------] 2 tests from Example
[ RUN      ] Example.SampleTest_PASS
[       OK ] Example.SampleTest_PASS (0 ms)
[ RUN      ] Example.SampleTest_FAIL
/home/jeliser/code/github_sandbox/cmake/test/applications/example/sample_unit_test.cpp:10: Failure
Value of: false
  Actual: false
Expected: true
[  FAILED  ] Example.SampleTest_FAIL (0 ms)
[----------] 2 tests from Example (1 ms total)

[----------] Global test environment tear-down
[==========] 2 tests from 1 test case ran. (1 ms total)
[  PASSED  ] 1 test.
[  FAILED  ] 1 test, listed below:
[  FAILED  ] Example.SampleTest_FAIL

 1 FAILED TEST
```
