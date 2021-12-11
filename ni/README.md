```bash
docker build . ni-drivers:latest -f .Dockerfile
docker_bash --privileged --cap-add=ALL -v /lib/modules:/lib/modules ni-drivers:latest
```
