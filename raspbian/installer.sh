#!/bin/bash

ROOT=$( pwd )

sudo apt install -y portaudio19-dev python-pyaudio python3-pyaudio

if [[ $( cat /boot/config.txt | grep dtoverlay | wc -l ) -eq 0 ]]; then
  echo "Setting up the GPIO"
  echo "dtoverlay=spi0-1cs,cs0_pin=25" | sudo tee -a /boot/config.txt
fi

# Installing the base python packages
pip install -q -r requirements.txt 

# https://github.com/pololu/drv8835-motor-driver-rpi
if [[ $( pip freeze | grep pololu-drv8835-rpi | wc -l ) -eq 0 ]]; then
  echo "Installing - Pololu DRV8835 Dual Motor Driver Kit for Raspberry Pi"

  TMP=/tmp/drv8835-motor-driver-rpi

  rm -rf ${TMP} && \
  git clone https://github.com/pololu/drv8835-motor-driver-rpi.git ${TMP} && \
  cd ${TMP} && \
  python setup.py install
  cd ${ROOT}
fi

