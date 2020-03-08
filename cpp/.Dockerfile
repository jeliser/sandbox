FROM ubuntu:18.04

RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential libwebsockets-dev \
  && apt-get -y autoremove && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/cpp/
COPY . .
RUN make -j8
