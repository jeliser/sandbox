#!/usr/bin/env python
import nidaqmx
import time
import argparse
import traceback


parser = argparse.ArgumentParser(description='Publishes rows of data.')
args = parser.parse_args()

for name in nidaqmx.system.System().devices.device_names:
  device = nidaqmx.system.Device(name)
  print([name, device.product_category, device.product_num, device.product_type])

  print('AI size: ', len(device.ai_physical_chans))
  for ai in device.ai_physical_chans:
    print('AI: ', ai)

try:
  with nidaqmx.Task() as task:
    for chan in nidaqmx.system.Device('cDAQ2Mod1').ai_physical_chans:
      task.ai_channels.add_ai_voltage_chan(chan.name)
    for i in range(0,10):
      print(task.read())
      time.sleep(1)
except KeyboardInterrupt:
  pass
except:
    traceback.print_exc()
