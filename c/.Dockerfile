FROM ubuntu:18.04

RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential libmosquitto-dev libwebsockets-dev libqpid-proton8-dev \
  && apt-get -y autoremove && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/c/
COPY . .
RUN make -j8
