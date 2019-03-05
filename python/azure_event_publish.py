import sys
import logging
import datetime
import time
import os
import socket

from azure.eventhub import EventHubClient, Sender, EventData

logger = logging.getLogger("azure")

# Address can be in either of these formats:
# "amqps://<URL-encoded-SAS-policy>:<URL-encoded-SAS-key>@<mynamespace>.servicebus.windows.net/myeventhub"
# "amqps://<mynamespace>.servicebus.windows.net/myeventhub"
# For example:
#ADDRESS = "amqps://publisher:oVlucuH5UnDibCbCWGg+V1ksLTkVfKfPFNQO1upnjPU=@kelvin-flight.servicebus.windows.net/example"
#ADDRESS = "amqps://publisher:oVlucuH5UnDibCbCWGg+V1ksLTkVfKfPFNQO1upnjPU=@kelvin-flight.servicebus.windows.net/example"
ADDRESS = "amqps://kelvin-flight.servicebus.windows.net/example"
##Endpoint=sb://kelvin-flight.servicebus.windows.net/;SharedAccessKeyName=publisher;SharedAccessKey=vtJwe9BVGMY7qunt+hhbCHGKlG+zFl8HIdavSS0yTeE=;EntityPath=example

# SAS policy and key are not required if they are encoded in the URL
USER = None
KEY = None
USER = "publisher"
KEY = "vtJwe9BVGMY7qunt+hhbCHGKlG+zFl8HIdavSS0yTeE="

HOST='127.0.0.1'
PORT=9999

try:
    if not ADDRESS:
        raise ValueError("No EventHubs URL supplied.")

    # Create Event Hubs client
    client = EventHubClient(ADDRESS, debug=True, username=USER, password=KEY)
    sender = client.add_sender(partition="0")
    client.run()
    try:
        start_time = time.time()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))
        s.listen(1)
        conn, addr = s.accept()
        print('Connected by', addr)
        count
        while True:
            data = conn.recv(1024)
            if not data:
                break
            sender.send(EventData(data))
            count = count + 1
            if(count % 30 == 0):
                print('Uploaded %d messages' % (count))
        s.close()
    except:
        raise
    finally:
        end_time = time.time()
        client.stop()
        run_time = end_time - start_time
        logger.info("Runtime: {} seconds".format(run_time))

except KeyboardInterrupt:
    pass
