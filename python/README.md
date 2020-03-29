``` bash
> socat tcp-l:8888,reuseaddr,fork tcp-l:5555,reuseaddr,fork
> ./csv_publisher.py ~/downloads/usws-van.csv
> ./rebroadcast.py
> nc 127.0.0.1 7777
```
