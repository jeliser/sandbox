FROM ubuntu:18.04

RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential \
  && apt-get -y autoremove && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/cpp/
COPY . .
RUN make fibonacci -j8

CMD /tmp/cpp/bin/fibonacci
