FROM ubuntu:18.04

RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential cmake \
  && apt-get -y autoremove && rm -rf /var/lib/apt/lists/*

COPY ./ /tmp/cmake/
RUN cd /tmp/cmake && make -j8
