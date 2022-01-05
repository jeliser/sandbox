``` bash
docker build . -t nginx:latest -f .Dockerfile
docker run -it --tmpfs /tmp --tmpfs /run --tmpfs /run/lock -v /sys/fs/cgroup:/sys/fs/cgroup:ro --stop-signal SIGRTMIN+3 -p 5000:80 nginx:latest /usr/sbin/init
```

``` bash
firefox 127.0.0.1:5000
```
