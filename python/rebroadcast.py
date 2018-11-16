#!/usr/bin/env python
import select
import socket

source = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
source.connect(('192.168.0.43', 5555))
print "Connecting the SOURCE on port 5555"

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('', 7777))
server.listen(20)
print "Listening on port 7777"

source_list = [server, source]
read_list = []
while True:
    readable, writable, errored = select.select(read_list + source_list, [], [])
    for s in readable:
        if s is server:
            client_socket, address = server.accept()
            read_list.append(client_socket)
            print "Accepted connection from", address
        elif s is source:
            data = s.recv(1024)
            if data:
                for out in read_list:
                    out.send(data)
        else:
            data = s.recv(1024)
            if not data:
                s.close()
                read_list.remove(s)
                print "Removed connection"
