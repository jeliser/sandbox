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
> ./run_object ../lib/libexample.so
```
