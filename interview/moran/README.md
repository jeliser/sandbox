# C++ Coding Challenge

## Problem Description

A [**Harshad number**](https://en.wikipedia.org/wiki/Harshad_number) is a number which is divisible by the sum of its digits. For example, `132` is divisible by `6` (1+3+2).  A subset of the **Harshad numbers** are the **Moran numbers**. **Moran numbers** yield a prime when divided by the sum of their digits. For example, `133` divided by `7` (1+3+3) yields `19`, a prime.

### Requirements

Create a C++ class with a templatized method that takes an input and returns:
- `"M"` if the input is a **Moran number**
- `"H"` if the input is a **Harshad number** (non-Moran)
- `"N"` if the input is neither.

The class should be able to process `uint16_t`, `int32_t`, `std::string`, and `float` input variables as template types.  Floating point numbers should be rounded to the nearest integer value to perform the calculation.

Unit tests should be created to validate that the class is able to correctly process inputs from each of the C++ types specified above (`uint16_t`, `int32_t`, `std::string`, and `float`).

### Bonus

The main application should accept command line arguments for the inputs to process.  It's up to the developer on how to accept the command line inputs (formatting, etc), but there should be a `--help` option with the explanation and the `README.md` should be updated with example usages.

### Example

```
moran(132) ➞ "H"
moran(133.25) ➞ "M"
moran("134") ➞ "N"
```

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
