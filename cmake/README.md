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
> cd build
> make
> cd bin

[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/cmake/build/bin]  (git:master:e166b5b) 
 > ./run_object ../lib/libexample.so
Loading shared object: ../lib/libexample.so - SUCCESS
Failed to find 'ctest1' - ../lib/libexample.so: undefined symbol: ctest1
Found 'hello_world' - 12345
Found 'newInstance' - hello world this is a list
Closing shared object: ../lib/libexample.so - SUCCESS

```
