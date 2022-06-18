#!/bin/bash

ROOT=$( pwd )

# https://github.com/pololu/drv8835-motor-driver-rpi
if [[ $( pip freeze | grep wiringpi | wc -l ) -eq 0 ]]; then
  echo "Installing - Pololu DRV8835 Dual Motor Driver Kit for Raspberry Pi"
  pip install wiringpi pigpio

  git clone https://github.com/pololu/drv8835-motor-driver-rpi.git /tmp/drv8835-motor-driver-rpi && \
  cd /tmp/drv8835-motor-driver-rpi && \
  python setup.py install
  cd ${ROOT}
fi

