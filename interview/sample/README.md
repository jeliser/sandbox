# C++ Coding Challenge

This is the stubbed out framework that the application challenge will be delivered in.  This is a great chance to
familiarize yourself with running and testing the commands that we'll use in the evulation process.  You can also use this
sample to test out the **Docker** and **gtest** tools used in the coding challenge.

## Problem Description

The problem **description** will be here.

### Requirements

The **requirements** will be here.

### Bonus

The **bonus** will be here.

### Example

The **example** outputs will be here.

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
