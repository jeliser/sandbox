#!/usr/bin/env python
import paho.mqtt.client as mqttClient
import time
import struct

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected                #Use global variable
        Connected = True                #Signal connection 
    else:
        print("Connection failed")
                    
Connected = False   #global variable for the state of the connection
 
broker_address= "127.0.0.1"  #Broker address
port = 1883                         #Broker port
 
client = mqttClient.Client("PythonPub")            #create new instance
client.on_connect= on_connect                      #attach function to callback
client.connect(broker_address, port=port)          #connect to broker
client.loop_start()        #start the loop
 
while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
try:
    with open('sparkplug_data.raw', 'rb') as f:
        raw = 1
        while raw:
            payload_len = struct.unpack('I', f.read(4))[0]
            raw = struct.unpack('%ds' % (payload_len), f.read(payload_len))[0]
            client.publish('replay', raw)
            time.sleep(1)
 
except KeyboardInterrupt:
    print "exiting"
    client.disconnect()
    client.loop_stop()
