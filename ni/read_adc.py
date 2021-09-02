#!/usr/bin/env python
import nidaqmx
import argparse
import traceback


parser = argparse.ArgumentParser(description='Publishes rows of data.')
args = parser.parse_args()

try:
  with nidaqmx.Task() as task:
    task.ai_channels.add_ai_voltage_chan('Dev1/ai0')
    task.read()
except:
    traceback.print_exc()

