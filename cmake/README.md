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
Scanning dependencies of target run_object
[ 16%] Building CXX object common/CMakeFiles/run_object.dir/application/application.cpp.o
[ 33%] Linking CXX executable ../bin/run_object
[ 33%] Built target run_object
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
 > ./run_object ../lib/libexample.so
Loading shared object: ../lib/libexample.so - SUCCESS
Failed to find 'ctest1' - ../lib/libexample.so: undefined symbol: ctest1
Found 'hello_world' - 12345
Found 'newInstance' - hello world this is a list
Closing shared object: ../lib/libexample.so - SUCCESS

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:0e61139) 
 > ./run_object ../lib/libanother_example.so
Loading shared object: ../lib/libanother_example.so - SUCCESS
Failed to find 'ctest1' - ../lib/libanother_example.so: undefined symbol: ctest1
Found 'hello_world' - 12345
Found 'newInstance' - hello world this is a list
Closing shared object: ../lib/libanother_example.so - SUCCESS

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:0e61139) 
 > ./run_object
Usage: ./run_object [shared_object_to_load]

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:0e61139) 
 > ./run_object made_up_lib
Loading shared object: made_up_lib - FAILED (made_up_lib: cannot open shared object file: No such file or directory)

```
