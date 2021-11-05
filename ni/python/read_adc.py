#!/usr/bin/env python
import nidaqmx
import time
import argparse
import traceback


parser = argparse.ArgumentParser(description='Publishes rows of data.')
args = parser.parse_args()

help(nidaqmx.system)
print(nidaqmx.system.device)

try:
  with nidaqmx.Task() as task:
    task.ai_channels.add_ai_voltage_chan('Dev2/ai0')
    for i in range(0,10):
      print(task.read())
      time.sleep(1)
except KeyboardInterrupt:
  pass
except:
    traceback.print_exc()

