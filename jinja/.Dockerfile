FROM ubuntu:18.04

RUN apt-get update && apt-get install -y --no-install-recommends \
  python3 python3-pip \
  && apt-get -y autoremove && rm -rf /var/lib/apt/lists/*
RUN ln -s /usr/bin/python3 /usr/bin/python

RUN pip3 install -U setuptools wheel
RUN pip3 install -U jinja2 pyaml

WORKDIR /tmp/jinja
COPY . .
RUN ./generate_files.py
