#!/usr/bin/env python
import argparse
import time
import progressbar

'''
> pip install progressbar
'''

def main():
  parser = argparse.ArgumentParser(description='Publishes rows of data.')
  parser.add_argument('-s', '--sleep', help='The number of seconds to sleep between updates)', default=0.5, type=float)
  parser.add_argument('-d', '--duration', help='The number of seconds to run the example)', default=10.0, type=float)
  args = parser.parse_args()

  intervals = int(args.duration / args.sleep)

  bar = progressbar.ProgressBar(maxval=intervals, widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])
  bar.start()
  try:
    for i in range(intervals):
      bar.update(i+1)
      time.sleep(args.sleep)
  finally:
    bar.finish()

if __name__ == '__main__':
  main()

