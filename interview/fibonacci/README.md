
## Build

``` bash
 > make
docker build -t interview-cpp:latest .
Sending build context to Docker daemon  75.26kB
Step 1/6 : FROM ubuntu:18.04
 ---> c3c304cb4f22
Step 2/6 : RUN apt-get update && apt-get install -y --no-install-recommends   build-essential   && apt-get -y autoremove && rm -rf /var/lib/apt/lists/*
 ---> Using cache
 ---> 2db116703b4f
Step 3/6 : WORKDIR /tmp/cpp/
 ---> Using cache
 ---> d57983ddbf36
Step 4/6 : COPY . .
 ---> 7f3967719181
Step 5/6 : RUN make fibonacci -j8
 ---> Running in b0464e0f8011
g++ -std=c++14 -g  ./src/fibonacci.cpp -o ./bin/fibonacci
Removing intermediate container b0464e0f8011
 ---> da01794766f1
Step 6/6 : CMD /tmp/cpp/bin/fibonacci
 ---> Running in 024a65cc98f2
Removing intermediate container 024a65cc98f2
 ---> de1b95bd5d72
Successfully built de1b95bd5d72
Successfully tagged interview-cpp:latest
```

## Run

``` bash
 > docker run interview-cpp
Hello World!
```

