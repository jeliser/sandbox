# C++ Coding Challenge

## Execution

## Build

The following `make` command will build the application and the unit test.

``` bash
 > make
docker build -t interview-cpp:latest .
.
.
Successfully built de1b95bd5d72
Successfully tagged interview-cpp:latest
```

Or you can run the `docker` command manually.

```bash
> docker build -t interview-cpp:latest .
.
.
Successfully built de1b95bd5d72
Successfully tagged interview-cpp:latest
```

## Run

The following `docker` command will execute the application.

``` bash
 > docker run interview-cpp
Hello World!
```

## Test

The following `docker` command will execute the unit tests.

``` bash
> docker run --rm -it --entrypoint /tmp/cpp/bin/test interview-cpp
[==========] Running 0 tests from 0 test cases.
[==========] 0 tests from 0 test cases ran. (0 ms total)
[  PASSED  ] 0 tests.
```
