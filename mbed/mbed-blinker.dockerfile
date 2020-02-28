FROM mbed-os

WORKDIR /mbed
RUN mbed-cli import http://github.com/ARMmbed/mbed-os-example-blinky

WORKDIR /mbed/mbed-os-example-blinky
RUN \
  mbed-cli ls && \
  mbed-cli config --list && \
  mbed-cli compile
