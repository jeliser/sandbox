
# Reference

[Yocto 3.14 LTS docs](https://www.yoctoproject.org/docs/3.1.4/mega-manual/mega-manual.html)
[Yocto 3.14 LTS ubuntu packages](https://www.yoctoproject.org/docs/3.1.4/mega-manual/mega-manual.html#ubuntu-packages)
A good quick [reference](https://ubs_csse.gitlab.io/secu_os/tutorials/crops_yocto.html)

# Running the container on a host machine

The following will start up the docker container using the local file system and let you build
the yocto image on the host machine.  This can be used as a build helper so that you can use
the normal tooling on your host machine for development and the build happens inside the container
and the artifacts are loaded directly on the host machine.

```
docker run --rm -it -v $(pwd):$(pwd) crops/poky --workdir=$(pwd)
```
