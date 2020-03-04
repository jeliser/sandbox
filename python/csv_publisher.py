#!/usr/bin/env python
import socket
import os
import time

filename = '~/downloads/usws-van.csv'

source_ip   = '127.0.0.1'
source_port = 6666
print('Attempting to establish connection to {}:{} to replay file {}'.format(source_ip, source_port, filename))
source = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
source.connect((source_ip, source_port))

with open(os.path.abspath(os.path.expanduser(filename))) as fp:
    header = fp.readline()
    print('CSV header: {}'.format(header.split(',')))
    row = fp.readline()
    while row:
        source.send(row)
        time.sleep(1.0)
        row = fp.readline()

