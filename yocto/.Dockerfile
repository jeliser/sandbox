FROM ubuntu:20.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  gawk wget git-core diffstat unzip texinfo gcc-multilib \
  build-essential chrpath socat cpio python3 python3-pip python3-pexpect \
  xz-utils debianutils iputils-ping python3-git python3-jinja2 libegl1-mesa libsdl1.2-dev \
  pylint3 xterm python3-subunit mesa-common-dev file \
  vim
#  && apt-get -y autoremove && rm -rf /var/lib/apt/lists/*
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y locales

# Set the locale
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8  

# Create a non-root user
RUN useradd -ms /bin/bash yoctouser
USER yoctouser
WORKDIR /home/yoctouser

# Clone the Yocto repo and the LTS release
ARG YOCTO_VERSION=3.1.4
RUN git clone git://git.yoctoproject.org/poky
WORKDIR poky
RUN git checkout tags/yocto-${YOCTO_VERSION} -b yocto-${YOCTO_VERSION}


#RUN . $( pwd )/oe-init-build-env && bitbake
RUN . $( pwd )/oe-init-build-env && bitbake core-image-minimal

