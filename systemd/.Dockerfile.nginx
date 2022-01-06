FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update
RUN apt-get install -y --no-install-recommends nginx-light systemctl
RUN systemctl start nginx && systemctl enable nginx
RUN apt-get install -y init

