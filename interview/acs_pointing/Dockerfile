FROM ubuntu:20.04

RUN apt update && apt install -y --no-install-recommends \
  build-essential libgtest-dev \
  && apt -y autoremove && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/cpp/
COPY . .
RUN make release -j4
RUN make test -j4

CMD /tmp/cpp/bin/release
