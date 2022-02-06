#!/usr/bin/env python
import os
import sys
import argparse
import traceback
import csv
import glob
import numpy as np
import pandas as pd


def main():
  parser = argparse.ArgumentParser(description='Publishes rows of data.')
  parser.add_argument('-i', '--input-dir', help='The directory with the raw CSV files to be processed', required=True)
  parser.add_argument('-o', '--output-dir', help='The directory to place the converted CSV data', required=True)
  parser.add_argument('-v', '--verbose', help='Prints out the verbose processing information', action='store_true', default=False)
  parser.add_argument(      '--verbose-help', help='Prints out the verbose help menu', action='store_true', default=False)
  args = parser.parse_args()

  args.input_dir = os.path.join(os.path.abspath(args.input_dir), '')
  args.output_dir = os.path.join(os.path.abspath(args.output_dir), '')

  os.makedirs(args.output_dir, exist_ok=True)

  # Check if we're just using the verbose output
  if args.verbose_help:
    print('More help')
    sys.exit()

  try:
    for name in glob.glob(f'{args.input_dir}/**/*.csv', recursive=True):
      ignore = ['LTF', 'MEG', 'Purge']
      if any(filter(lambda i: i in name, ignore)):
        continue
      # Read the data into a pandas data frame and remove all the duplicate lines to only leave the latest updated row.
      df=pd.read_csv(name, sep=',', low_memory=False)
      if('Systems.csv' in name or 'GNCA.csv' in name):
        opath = os.path.join(args.output_dir, os.path.basename(name).split('_')[0])
        print(f'Processing: {name.split(args.input_dir)[-1]} to {opath}')
        
        # We are going to filter on any SUBSEC column in the CSV.  That's where the data will be aligned for the current timestep
        keys = list(filter(lambda k: 'SUBSEC' in k, df.columns))
        if any(keys):
          if args.verbose:
            print(f'Filtering on {keys[0]} in {name.split(args.input_dir)[-1]}')

          ## np.where and np.roll are apparently the key
          ## https://stackoverflow.com/questions/19125661/find-index-where-elements-change-value-numpy
          v = df[keys[0]]  # TODO: This should verify that it's only a single key been found
          idx = np.where(np.roll(v,-1)!=v)[0]
          fdf = df.iloc[idx]

          os.makedirs(opath, exist_ok=True)
          fdf.to_csv(os.path.join(opath, os.path.basename(name)), index=False)
    pass
  except KeyboardInterrupt: 
    running = False
  except:
    traceback.print_exc()


if __name__ == '__main__':
  main()

