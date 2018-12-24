#!/usr/bin/env python3
import struct
import matplotlib.pyplot as plot
import numpy as np

entries = dict()

xmin = 999999999999999999999
xmax = 0

# Read in all the data from the sample file
with open('./sample.raw', 'rb') as f:
    while True:
        raw = f.read(14)
        if len(raw) != 14:
            break
        row = struct.unpack('=IbQx', raw)
        if row[0] not in entries:
            entries[row[0]] = []
        entries[row[0]].append(row[1:])

# Perform some analysis on the loaded data
for entry, rows in entries.items():
    (y, x) = zip(*rows)  # Just save the data in the converted format
    xmax = max(xmax, max(x))
    xmin = min(xmin, min(x))

# Generate the subplots
h, p = plot.subplots(len(entries.keys()), sharex=True, sharey=True)
i = 0
for entry, rows in entries.items():
    (y, x) = zip(*rows)
    # Convert to seconds, and plot the step function
    p[i].step([(v - xmin) * 1.0E-9 for v in x], y)
    plot.yticks([1, 2], ('Start', 'Stop'))
    plot.ylim((0, 3))
    i += 1

plot.show()

