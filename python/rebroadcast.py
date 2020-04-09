#!/usr/bin/env python
import select
import socket
import argparse
import sys
import traceback
import time

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
RECONNECTION_TIMEOUT = 10.0


def clean_up(sock, name='unspecified'):
    try:
        if sock:
            print('Closed socket: {}'.format(name))
            sock.shutdown(socket.SHUT_RDWR)
            sock.close()
    except:
        traceback.print_exc()
    return None


def main():
    parser = argparse.ArgumentParser(description='Publishes rows of data.')
    parser.add_argument(      '--source-ip', help='The IP of the server to connect to (default: {})'.format(SOURCE_IP), default=SOURCE_IP)
    parser.add_argument(      '--source-port', help='The PORT of the server to connect to (default: {})'.format(SOURCE_PORT), default=SOURCE_PORT, type=int)
    parser.add_argument('-i', '--server-ip', help='The IP we want to rebroadcast from (default: {})'.format(SERVER_IP if SERVER_IP else 'INADDR_ANY'), default=SERVER_IP)
    parser.add_argument('-p', '--server-port', help='The PORT we want to rebroadcast from (default: {})'.format(SERVER_PORT), default=SERVER_PORT, type=int)
    parser.add_argument('-s', '--simultaneous-connections', help='The numer of SIMULTANEOUS CONNECTIONS to server from the rebroadcaster (default: {})'.format(SIMULTANEOUS_CONNECTIONS), default=SIMULTANEOUS_CONNECTIONS, type=int)
    parser.add_argument('-t', '--reconnection-timeout', help='The time in seconds between RECONNECTION ATTEMPTS (default: {})'.format(RECONNECTION_TIMEOUT), default=RECONNECTION_TIMEOUT, type=float)
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
    running = True

    # The main loop
    while running:
        try:
            # Go ahead and open up the server connection
            while not server:
                try:
                    print('Rebroadcasting to {}:{}'.format(args.server_ip if args.server_ip else 'INADDR_ANY', args.server_port))
                    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
                    server.bind((args.server_ip, args.server_port))
                    server.listen(0) # We want a backlog of zero connections
                except:
                    server = clean_up(server, 'server')
                    time.sleep(args.reconnection_timeout)

            # Make sure we are connected to the socket providing data before continuing to the main event loop
            while not source:
                try:
                    print('Attempting to establish connection to {}:{} for establishing the rebroadcast'.format(args.source_ip, args.source_port))
                    source = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    source.connect((args.source_ip, args.source_port))
                except:
                    source = clean_up(source, 'source')
                    time.sleep(args.reconnection_timeout)

            # While there are no socket problems, loop and dispatch all the incoming data to the listeners
            source_list = [server, source]
            failed = False
            while not failed:
                readable, writable, errored = select.select(serve_list + source_list, [], [])
                for s in readable:
                    # Handle the incoming server connection requests.
                    if s is server:
                        ## What the proper way to reject connections when the list is full?
                        client_socket, address = server.accept()
                        if len(serve_list) < args.simultaneous_connections:
                            serve_list.append(client_socket)
                            serve_lookup[client_socket] = address
                            print('Accepted connection from {}'.format(address))
                        else:
                            print('Ignored connection from {} ... maximum number of concurrent connections ({}) accepted.'.format(address, args.simultaneous_connections))
                            clean_up(client_socket, address)

                    # Handle the socket providing data to the application
                    elif s is source:
                        try:
                            data = s.recv(4096)
                            if data:
                                for out in serve_list:
                                    out.send(data)
                            else:
                                print('Source socket ({}:{}) terminated our connection.'.format(args.source_ip, args.source_port))
                                failed = True
                        except socket.error, err:
                            print('Error encountered with source socket.({})'.format(err))
                            failed = True

                    # Handle the disconnecting of connected clients
                    else:
                        try:
                            data = s.recv(4096)
                        except socket.error:
                            data = None

                        if data:
                            print('Ignoring data from {}: "{}"'.format(serve_lookup.pop(s, 'unknown'), data))
                        else:
                            print('Removing connection: {}'.format(serve_lookup.get(s, 'unknown')))
                            clean_up(s, serve_lookup.pop(s, 'unknown'))
                            serve_list.remove(s)

            # If there was a failure on the source socket, go ahead and clean it up and re-establish the connection
            if failed:
                source = clean_up(source, 'source') if source else None
                time.sleep(args.reconnection_timeout)

        except KeyboardInterrupt: 
            running = False
        except:
            traceback.print_exc()

    # Cleanup all the established connections
    for s in serve_list:
        clean_up(s, serve_lookup.pop(s, 'unknown'))
    serve_list = []

    ### Ugh, I'm trying to clean up the sockets properly, but if there are served connections, our bound socket doesn't get released properly.
    server = clean_up(server, 'server') if server else None
    source = clean_up(source, 'source') if source else None

    # Give the sockets a second to clean up
    time.sleep(0.25)

if __name__ == '__main__':
  main()

