# Install CMake

```bash
 > sudo apt-get install cmake
```

# Generate the Makefiles

```bash
 > cmake -H. -Bbuild
```

# Run the executeable

```bash
 > cd build
 > make
 > cd bin
 > ./run_object ../lib/libexample.so
```
