#!/usr/bin/env python
import select
import socket

help_str = """
############
## First
##   You can run a TCP server
##    > nc -l 5555
##   You can also run a proxy
##    > socat tcp-l:5555,reuseaddr,fork tcp-l:6666,reuseaddr,fork
##
## Second
##   You can run the rebroadcaster
##    > ./rebroadcaster.py
###
## Third
#   You can run the CSV publisher
##    > ./csv_publisher
##   You can pipe data to the TCP server using this command
##    > cat ~/downloads/usws-van.csv | nc 127.0.0.1 6666
##
## Fourth
##   You can connect a listener to the rebroadcaster socket to see that data
##    > nc 127.0.0.1 7777
############
"""
print(help_str)

source_ip   = '192.168.0.43'
source_port = 5555
source_ip   = '127.0.0.1'
source_port = 5555
print('Attempting to establish connection to {}:{} for establishing the rebroadcast'.format(source_ip, source_port))
source = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
source.connect((source_ip, source_port))

server_ip   = ''
server_port = 7777
print('Rebroadcasting on port {}'.format(server_port))
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('', server_port))
server.listen(20)

source_list = [server, source]
read_list = []
while True:
    readable, writable, errored = select.select(read_list + source_list, [], [])
    for s in readable:
        if s is server:
            client_socket, address = server.accept()
            read_list.append(client_socket)
            print('Accepted connection from {}'.format(address))
        elif s is source:
            data = s.recv(1024)
            if data:
                for out in read_list:
                    out.send(data)
        else:
            try:
                data = s.recv(1024)
            except socket.error:
                data = None
            if not data:
                s.close()
                read_list.remove(s)
                print('Removed connection')
