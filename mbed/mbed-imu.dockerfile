FROM mbed-os

WORKDIR /mbed
RUN mbed-cli import https://os.mbed.com/teams/ST/code/HelloWorld_ST_Sensors

WORKDIR /mbed/HelloWorld_ST_Sensors
RUN \
  mbed-cli ls && \
  mbed-cli config --list && \
  mbed-cli add https://github.com/ARMmbed/mbed-os && \
  mbed-cli deploy && \
  mbed-cli compile -m DISCO_L475VG_IOT01A -t GCC_ARM -j8 -v
