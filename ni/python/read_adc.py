#!/usr/bin/env python
import nidaqmx
import time
import argparse
import traceback
import yaml

parser = argparse.ArgumentParser(description='Publishes rows of data.')
args = parser.parse_args()

class NIDevice():
  def __init__(self, name):
    self.device = nidaqmx.system.Device(name)

    print([name, self.device.product_category, self.device.product_num, self.device.product_type])
    #print([name, self.device.product_category, self.device.accessory_product_nums, self.device.accessory_product_types, self.device.accessory_serial_nums])

    print('AI size: ', len(self.device.ai_physical_chans))
    for ai in self.device.ai_physical_chans:
      print(ai.name)
      print('AI: ', ai)
    print([ai.name for ai in self.device.ai_physical_chans])

    print('AO size: ', len(self.device.ao_physical_chans))
    for ao in self.device.ao_physical_chans:
      print(ao.name)
      print('AO: ', ao)
    print([ao.name for ao in self.device.ao_physical_chans])

    print('DI size: ', len(self.device.di_ports))
    for di in self.device.di_ports:
      print(di.name)
      print('DI: ', di)
    print([di.name for di in self.device.di_ports])

    print('DO size: ', len(self.device.do_ports))
    for do in self.device.do_ports:
      print(do.name)
      print('DO: ', do)
    print([do.name for do in self.device.do_ports])

    #help(self.device)

for name in nidaqmx.system.System().devices.device_names:
  NIDevice(name)

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

