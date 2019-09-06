#!/usr/bin/env python3

import asyncio
import random
import time

async def coro(tag):
    print(">", tag)
    await asyncio.sleep(random.uniform(1, 3))
    print("<", tag)
    return tag

loop = asyncio.get_event_loop()

group1 = asyncio.gather(*[coro("group 1.{}".format(i)) for i in range(1, 6)])
group2 = asyncio.gather(*[coro("group 2.{}".format(i)) for i in range(1, 4)])
group3 = asyncio.gather(*[coro("group 3.{}".format(i)) for i in range(1, 10)])
groups = [group1, group2, group3]

all_groups = asyncio.gather(*groups)

results = None
try:
    print('Delaying execution')
    asyncio.run(asyncio.sleep(1.0))
    loop.create_task(coro("Starts automatically"))
    print('Delaying execution another way')
    time.sleep(1.0)
    results = loop.run_until_complete(all_groups)
except KeyboardInterrupt as e:
    results = "Caught keyboard interrupt. Canceling tasks..."
    for group in groups:
        group.cancel()
    all_groups.cancel()
finally:
    loop.close()

print(results)

