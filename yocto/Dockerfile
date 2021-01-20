FROM crops/poky

# The current user doesn't own the directory, root does.  Change the ownership
USER root
RUN chown -R usersetup:usersetup ./
USER usersetup

RUN git clone git://git.yoctoproject.org/poky
WORKDIR poky

RUN source $( pwd )/oe-init-build-env && bitbake core-image-minimal
