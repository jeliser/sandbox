#!/usr/bin/env python
import select
import socket
import argparse
import sys
import traceback

HELP = """
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
##
## Third
##   You can run the CSV publisher
##    > ./csv_publisher
##   You can pipe data to the TCP server using this command
##    > cat ~/downloads/usws-van.csv | nc 127.0.0.1 6666
##
## Fourth
##   You can connect a listener to the rebroadcaster socket to see that data
##    > nc 127.0.0.1 7777
############
"""

SOURCE_IP   = '127.0.0.1'
SOURCE_PORT = 5555
SERVER_IP   = ''
SERVER_PORT = 7777
SIMULTANEOUS_CONNECTIONS = 20

parser = argparse.ArgumentParser(description='Publishes rows of data.')
parser.add_argument(      '--source-ip', help='The IP of the server to connect to (default: {})'.format(SOURCE_IP), default=SOURCE_IP)
parser.add_argument(      '--source-port', help='The PORT of the server to connect to (default: {})'.format(SOURCE_PORT), default=SOURCE_PORT, type=int)
parser.add_argument('-i', '--server-ip', help='The IP we want to rebroadcast from (default: {})'.format(SERVER_IP if SERVER_IP else 'INADDR_ANY'), default=SERVER_IP)
parser.add_argument('-p', '--server-port', help='The PORT we want to rebroadcast from (default: {})'.format(SERVER_PORT), default=SERVER_PORT, type=int)
parser.add_argument('-s', '--simultaneous-connections', help='The numer of SIMULTANEOUS CONNECTIONS to server from the rebroadcaster (default: {})'.format(SIMULTANEOUS_CONNECTIONS), default=SIMULTANEOUS_CONNECTIONS, type=int)
parser.add_argument(      '--verbose-help', help='Prints out the verbose help menu', action='store_true', default=False)
args = parser.parse_args()

# Check if we're just using the verbose output
if args.verbose_help:
    print(HELP)
    sys.exit()

source = None
server = None
serve_list = []
serve_lookup = dict()

try:
    print('Attempting to establish connection to {}:{} for establishing the rebroadcast'.format(args.source_ip, args.source_port))
    source = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    source.connect((args.source_ip, args.source_port))

    print('Rebroadcasting to {}:{}'.format(args.server_ip if args.server_ip else 'INADDR_ANY', args.server_port))
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((args.server_ip, args.server_port))
    server.listen(0) # We want a backlog of zero connections

    source_list = [server, source]
    while True:
        readable, writable, errored = select.select(serve_list + source_list, [], [])
        for s in readable:
            if s is server:
                ## What the proper way to reject connections when the list is full?
                client_socket, address = server.accept()
                if len(serve_list) < args.simultaneous_connections:
                    serve_list.append(client_socket)
                    serve_lookup[client_socket] = address
                    print('Accepted connection from {}'.format(address))
                else:
                    print('Ignored connection from {} ... maximum number of concurrent connections ({}) accepted.'.format(address, args.simultaneous_connections))
                    client_socket.shutdown(socket.SHUT_RDWR)
                    client_socket.close()
            elif s is source:
                data = s.recv(1024)
                if data:
                    for out in serve_list:
                        out.send(data)
            else:
                try:
                    data = s.recv(1024)
                except socket.error:
                    data = None
                if not data:
                    s.close()
                    serve_list.remove(s)
                    print('Removed connection: {}'.format(serve_lookup.pop(s, 'unknown')))
except KeyboardInterrupt: 
    pass
except:
    traceback.print_exc()
finally:

    ### Ugh, I'm trying to clean up the sockets properly, but if there are served connections, our bound socket doesn't get released properly.

    for s in serve_list:
        try:
            print('Closed connection: {}'.format(serve_lookup.pop(s, 'unknown')))
            s.shutdown(socket.SHUT_RDWR)
            s.close()
        except:
            traceback.print_exc()
    serve_list = []

    if server:
        try:
            print('Closed server connection')
            server.shutdown(socket.SHUT_RDWR)
            server.close()
            server = None
        except:
            traceback.print_exc()

    if source:
        try:
            print('Closed source connection')
            source.shutdown(socket.SHUT_RDWR)
            source.close()
            source = None
        except:
            traceback.print_exc()

