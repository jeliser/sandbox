FROM ubuntu:18.04

RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential vim git wget python3-dev python3-pip python3-distutils \
  && apt-get remove -y wget && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN pip3 install setuptools wheel
RUN pip3 install mbed-cli

