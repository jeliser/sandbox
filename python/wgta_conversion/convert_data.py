#!/usr/bin/env python
import os
import sys
import argparse
import traceback
import csv
import glob
import json
import multiprocessing
import numpy as np
import pandas as pd
from scipy import io

'''
Method for processing the files into better data structures
'''
def process_file(tup):
  (args, name) = tup

  # Scenarios that should be ignored
  ignore = ['Purge', 'LTF03']
  if any(filter(lambda i: i in name, ignore)):
    return

  # Read the data into a pandas data frame and remove all the duplicate lines to only leave the latest updated row.
  df=pd.read_csv(name, sep=',', low_memory=False)
  # Automatically remove the system name from the column headers
  df.columns = [ col.split(':')[-1] for col in df.columns ]

  # We only want to process a subset of files 
  if any(filter(lambda n: n in name, ['Systems.csv', 'GNCA.csv'])):
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

      # Make the column names something easier to understand
      lookup = json.load(open('lookup.json'))
      df.rename(columns=lookup, inplace=True)

      # Display any of the columns that might still be too large for MATLAB column names
      failed = False
      for col in df.columns:
        if len(col) >= 31 or '.' in col:
          print(len(col), col)
          failed = True
      if failed:
        return
      # Extract the subset of data from the original data frame using the filtered indexes
      fdf = df.iloc[idx]

      # Create the output directory and write out all the file types
      os.makedirs(opath, exist_ok=True)
      fullpath = os.path.join(opath, os.path.splitext(os.path.basename(name))[0])
      fdf.to_csv(fullpath + '.csv', index=False)
      fdf.to_pickle(fullpath + '.pkl')
      try:
        io.savemat(fullpath + '.mat', {f'{os.path.basename(fullpath)}':fdf.to_dict('list')}, oned_as='column')
      except:
        traceback.print_exc()


def main():
  parser = argparse.ArgumentParser(description='Publishes rows of data.')
  parser.add_argument('-i', '--input-dir', help='The directory with the raw CSV files to be processed', required=True)
  parser.add_argument('-o', '--output-dir', help='The directory to place the converted CSV data', required=True)
  parser.add_argument('-j', '--pool', help='The number of threads to put in the thread pool (default: 4)', default=4)
  parser.add_argument('-v', '--verbose', help='Prints out the verbose processing information', action='store_true', default=False)
  parser.add_argument(      '--verbose-help', help='Prints out the verbose help menu', action='store_true', default=False)
  args = parser.parse_args()

  args.input_dir = os.path.join(os.path.abspath(args.input_dir), '')
  args.output_dir = os.path.join(os.path.abspath(args.output_dir), '')

  os.makedirs(args.output_dir, exist_ok=True)

  # Check if we're just using the verbose output
  if args.verbose_help:
    print('Add more help here')
    sys.exit()

  try:
    p = multiprocessing.Pool(args.pool)
    p.map(process_file, [(args, filename) for filename in sorted(glob.glob(f'{args.input_dir}/**/*.csv', recursive=True))])
  except KeyboardInterrupt: 
    running = False
  except:
    traceback.print_exc()


if __name__ == '__main__':
  main()

