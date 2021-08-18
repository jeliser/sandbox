#!/usr/bin/env python

# sudo apt-get install python3-tk

import serial
import time
import csv
import matplotlib
matplotlib.use("tkAgg")
import matplotlib.pyplot as plt
import numpy as np
import traceback

ser = serial.Serial('/dev/ttyUSB0', 112500)
ser.flushInput()

plot_window = 20
y_var = np.array(np.zeros([plot_window]))

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(y_var)

while True:
    try:
        ser_bytes = ser.readline()
        try:
            decoded_bytes = ser_bytes.decode('utf-8').strip().split(',')
        except:
            traceback.print_exc()
            continue

        '''
        with open("/tmp/test_data.csv","a") as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([time.time(),*decoded_bytes])
        '''

        y_var = np.append(y_var,np.float_(decoded_bytes)[1:])
        y_var = y_var[1:plot_window+1]
        line.set_ydata(y_var)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()
    except:
        traceback.print_exc()
        break
