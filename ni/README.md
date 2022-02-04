```bash
docker build . -t ni-drivers:latest -f .Dockerfile
docker_bash --privileged --cap-add=ALL -v /lib/modules:/lib/modules ni-drivers:latest

docker run -it --tmpfs /tmp --tmpfs /run --tmpfs /run/lock -v /sys/fs/cgroup:/sys/fs/cgroup:ro -v /lib/modules:/lib/modules --stop-signal SIGRTMIN+3 --name ni-runner ni-drivers:latest /usr/sbin/init
docker exec -it ni-runner /bin/bash
docker stop ni-runner
```

-------------------
# Working Instance of the NI module

## Host machine

### Stop the running NI services

``` bash
 > sudo systemctl stop ni-pxipf-nipxirm-bind
 > sudo systemctl stop nidrum
 > sudo systemctl stop nimxssvr                                        
 > sudo systemctl stop nipal                                           
 > sudo systemctl stop nipxicmsd                                       
 > sudo systemctl stop niroco                                          
 > sudo systemctl stop nisds                                           
 > sudo systemctl stop nisvcloc 
```

``` bash
[jeliser@jeliser-thinkpad-x1:~/code/sandbox/ni]  (git:master:4e3a22d) [ahead 1]
 > docker run -it --privileged --cap-add=ALL --tmpfs /tmp --tmpfs /run --tmpfs /run/lock -v /dev:/dev -v /sys/fs/cgroup:/sys/fs/cgroup:ro --name ni-runner --stop-signal SIGRTMIN+3 ni-drivers:latest /usr/sbin/init
 > docker run -it --privileged -v /dev:/dev -v /sys/fs/cgroup:/sys/fs/cgroup:ro --name ni-runner --stop-signal SIGRTMIN+3 ni-drivers:latest /usr/sbin/init
```

## Running in the container

``` bash
[jeliser@jeliser-thinkpad-x1:~/code/sandbox/ni]  (git:master:4e3a22d) [ahead 1]
 > docker exec -it ni-runner /bin/bash

root@069dd458a5d8:/tmp_ni/NILinux2021Q4DeviceDrivers# nilsdev -v
cDAQ1
   DevSerialNum: 0x1FCEB6C
   ProductType: cDAQ-9174
   cDAQ1Mod2
      CompactDAQ.ChassisDevName: cDAQ1
      CompactDAQ.SlotNum: 2
      DevSerialNum: 0x1FB99A6
      ProductType: NI 9474
   cDAQ1Mod4
      CompactDAQ.ChassisDevName: cDAQ1
      CompactDAQ.SlotNum: 4
      DevSerialNum: 0x1EF7423
      ProductType: NI 9213
   cDAQ1Mod3
      CompactDAQ.ChassisDevName: cDAQ1
      CompactDAQ.SlotNum: 3
      DevSerialNum: 0x1EF73B8
      ProductType: NI 9213
   cDAQ1Mod1
      CompactDAQ.ChassisDevName: cDAQ1
      CompactDAQ.SlotNum: 1
      DevSerialNum: 0x1FD1D48
      ProductType: NI 9201

```
