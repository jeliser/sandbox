#!/usr/bin/env python3

import multiprocessing
import time

def mp_worker(tup):
    (sleep, value) = tup
    time.sleep(int(sleep))
    print(" Process %s\tDONE" % value)

def mp_handler():
    p = multiprocessing.Pool(10)
    p.map(mp_worker, [(3, 'A'), (4, 'B'), (5, 'C'), (1, 'D'), (2, 'E'), (3, 'F')])

if __name__ == '__main__':
    mp_handler()

