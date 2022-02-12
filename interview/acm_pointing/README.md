# C++ Coding Challenge

This challenge was to build an Attitude Control Motor pointing application.  The user inputs delta coordinates and the application
will tell the user which planet the spacecraft is pointing to.

# Host Machine Information

The information on the host machine used to build the application locally and/or using Docker.

```bash
 > uname -r
5.13.0-28-generic

 > cat /etc/lsb-release 
DISTRIB_ID=Ubuntu
DISTRIB_RELEASE=20.04
DISTRIB_CODENAME=focal
DISTRIB_DESCRIPTION="Ubuntu 20.04.3 LTS"

 > gcc --version
gcc (Ubuntu 9.3.0-17ubuntu1~20.04) 9.3.0

 > docker --version
Docker version 20.10.12, build e91ed57
```

# Host Machine Build

There are no special requirements for building the application on a host machine and should work on most any Unix machine.  If you
plan to run the unit test on the host machine (`make test`), you will need to install the gtest library.  Refer to the included Dockerfile
for the Ubunutu package to use.

```bash
 > make
mkdir -p bin
g++ -std=c++14 -g -Isrc src/*.cpp -o bin/release

 > ./bin/release 
Input ACM movement (x y z) and press enter (ex: 3 -8 5), ctrl+c to exit:
3 4 5
Pointing Towards: GRACE at 3, 4, 5
-4 0 -6
Pointing Towards: MROW at -1, 4, -1
18 -5 3
Pointing Towards: BRAY at 17, -1, 2
```

# Docker Build

The application can be built using Docker and the following commands.  This is how I typically do my development and how it would be integrated in a CI/CD pipeline.
You also don't have to know the host machine setup in most cases to run the application.

``` bash
 > make docker && docker run --rm -it --entrypoint /tmp/cpp/bin/test interview-cpp && docker run --rm -it --entrypoint /tmp/cpp/bin/release interview-cpp 

docker build -t interview-cpp:latest .
[+] Building 5.0s (11/11) FINISHED
.
.
 => => exporting layers
 => => writing image sha256:c78b6d306744ee44686fe1c0a566e9d40dcce5acc5e0037e5df35fb79ced3e39
 => => naming to docker.io/library/interview-cpp:latest
[==========] Running 3 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 3 tests from ACM
[ RUN      ] ACM.SampleValidation
[       OK ] ACM.SampleValidation (0 ms)
[ RUN      ] ACM.ZeroValidation
[       OK ] ACM.ZeroValidation (0 ms)
[ RUN      ] ACM.PlanetValidation
[       OK ] ACM.PlanetValidation (0 ms)
[----------] 3 tests from ACM (0 ms total)

[----------] Global test environment tear-down
[==========] 3 tests from 1 test suite ran. (0 ms total)
[  PASSED  ] 3 tests.
Input ACM movement (x y z) and press enter (ex: 3 -8 5), ctrl+c to exit:
3 4 5
Pointing Towards: GRACE at 3, 4, 5
-4 0 -6
Pointing Towards: MROW at -1, 4, -1
18 -5 3
Pointing Towards: BRAY at 17, -1, 2
```


