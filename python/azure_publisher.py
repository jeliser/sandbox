#!/usr/bin/env python
import select
import socket

from azure.eventhub import EventHubClient, Sender, EventData

# Address can be in either of these formats:
# "amqps://<URL-encoded-SAS-policy>:<URL-encoded-SAS-key>@<mynamespace>.servicebus.windows.net/myeventhub"
# "amqps://<mynamespace>.servicebus.windows.net/myeventhub"
# For example:
ADDRESS = "amqps://kelvin-flight.servicebus.windows.net/example"
# SAS policy and key are not required if they are encoded in the URL
USER = "publisher"
KEY = "vtJwe9BVGMY7qunt+hhbCHGKlG+zFl8HIdavSS0yTeE="

try:
    PORT = 9999
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('', PORT))
    server.listen(20)
    print "Listening on port %d" % (PORT)

    # Create Event Hubs client
    client = EventHubClient(ADDRESS, debug=True, username=USER, password=KEY)
    sender = client.add_sender(partition="0")
    client.run()

    try:
        source_list = [server]
        read_list = []
        count = 0
        while True:
            readable, writable, errored = select.select(read_list + source_list, [], [])
            for s in readable:
                if s is server:
                    client_socket, address = server.accept()
                    read_list.append(client_socket)
                    print "Accepted connection from", address
                else:
                    data = s.recv(1024)
                    if not data:
                        s.close()
                        read_list.remove(s)
                        print "Removed connection"
                    sender.send(EventData(data))
                    count = count + 1
                    if(count % 30 == 0):
                        print('Uploaded %d messages' % (count))
    except:
        raise
    finally:
        client.stop()
except KeyboardInterrupt:
    pass
