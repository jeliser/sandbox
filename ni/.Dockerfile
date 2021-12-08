FROM ubuntu:20.04

RUN apt-get update && apt-get install -y --no-install-recommends \
  ca-certificates linux-headers-$(uname -r) systemctl \
  build-essential wget unzip

WORKDIR /tmp
RUN wget --no-check-certificate https://download.ni.com/support/softlib/MasterRepository/LinuxDrivers2021Q4/NILinux2021Q4DeviceDrivers.zip
RUN unzip NILinux2021Q4DeviceDrivers.zip
WORKDIR NILinux2021Q4DeviceDrivers
RUN apt install ./ni-ubuntu2004firstlook-drivers-2021Q4.deb

RUN apt-get update && apt-get install -y --no-install-recommends \
  ni-daqmx \
  && apt-get -y autoremove && rm -rf /var/lib/apt/lists/*

RUN dkms autoinstall
