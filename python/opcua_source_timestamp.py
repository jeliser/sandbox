#!/usr/bin/env python

import asyncio

from datetime import datetime, timezone

import asyncua
from asyncua import Client
from asyncua.ua import DataValue

async def test():
  client = Client("opc.tcp://localhost:48010")
  await client.connect()
  node = client.get_node("ns=2;s=rando_publisher.raw.int32_example")
  print(node)

  x = DataValue(asyncua.ua.Variant(int(100)))
  x.SourceTimestamp = None
  await node.set_value(x)  # works

  # add a timestamp
  x = DataValue(asyncua.ua.Variant(int(3333)))
  x.SourceTimestamp = datetime.now(timezone.utc)
  await node.set_value(x)  # fails

  # BadWriteNotSupported: The server does not support writing the combination of value, status and timestamps provided.(BadWriteNotSupported)

asyncio.run(test())
