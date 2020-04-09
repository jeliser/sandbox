#!/usr/bin/env python
import socket
import os
import time
import argparse
import traceback

IP    = '127.0.0.1'
PORT  = 8888
SLEEP = 1.0

parser = argparse.ArgumentParser(description='Publishes rows of data.')
parser.add_argument('filename', help='The file to be published')
parser.add_argument('-i', '--ip', help='The IP of the server to connect to (default: {})'.format(IP), default=IP)
parser.add_argument('-p', '--port', help='The PORT of the server to connect to (default: {})'.format(PORT), default=PORT, type=int)
parser.add_argument('-s', '--sleep', help='The time (in secs) to SLEEP between each publish (default: {})'.format(SLEEP), default=SLEEP, type=float)
args = parser.parse_args()

print('Attempting to establish connection to {}:{} to replay file {}'.format(args.ip, args.port, args.filename))
source = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
source.connect((args.ip, args.port))

try:
    fp = open(os.path.abspath(os.path.expanduser(args.filename)))
    try:
        header = fp.readline()
        print('CSV header: {}'.format(header.split(',')))
        row = fp.readline()
        while row:
            source.send(row)
            time.sleep(args.sleep)
            row = fp.readline()
    finally:
        fp.close()
except:
    traceback.print_exc()
finally:
    source.close()

print('Closing application')

