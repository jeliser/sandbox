#!/usr/bin/env python
import os
import sys
import argparse
import traceback
import csv
import glob
import yaml
import multiprocessing
import numpy as np
import pandas as pd
from scipy import io

'''
Save the data frame to the output files.
'''
def save_data(opath, name, df):
  # Create the output directory and write out all the file types
  os.makedirs(opath, exist_ok=True)
  fullpath = os.path.join(opath, os.path.splitext(os.path.basename(name))[0])
  df.to_csv(fullpath + '.csv', index=False)
  df.to_pickle(fullpath + '.pkl')
  try:
    io.savemat(fullpath + '.mat', {f'{os.path.basename(fullpath)}':df.to_dict('list')}, oned_as='column')
  except:
    traceback.print_exc()

'''
Method for processing the files into better data structures
'''
def process_file(tup):
  (args, name) = tup

  # Scenarios that should be ignored
  ignore = ['Purge', 'LTF03', 'MEG']
  if any(filter(lambda i: i in name, ignore)):
    return

  # We only want to process a subset of files 
  if any(filter(lambda n: n in name, ['Systems.csv', 'GNCA.csv'])):
    # Read the data into a pandas data frame and remove all the duplicate lines to only leave the latest updated row.
    df = pd.read_csv(name, sep=',', low_memory=False)
    # Automatically remove the system name from the column headers
    df.columns = [ col.split(':')[-1] for col in df.columns ]

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
      lookup = yaml.safe_load(open('lookup.yaml'))
      df.rename(columns=lookup, inplace=True)

      # Show any duplicate column names
      cols = list(df.columns)
      duplicates = set([x for x in cols if cols.count(x) > 1])
      if any(duplicates):
        print(duplicates)

      # Display any of the columns that might still be too large for MATLAB column names
      failed = False
      for col in df.columns:
        if len(col) >= 31 or '.' in col:
          print(len(col), col)
          failed = True
      if failed:
        return
      # Extract the subset of data from the original data frame using the filtered indexes
      save_data(opath, name, df.iloc[idx])

'''
Method for processing the files into better data structures
'''
def combine_file(tup):
  (args, path) = tup

  # Grab the files we are going to process
  files = sorted(glob.glob(f'{path}/*.csv'))
  systems = list(filter(lambda f: 'System' in f, files))[-1]
  gnc     = list(filter(lambda f: 'GNC' in f, files))[-1]

  # Align data between the Systems file and the GNC file
  df_systems = pd.read_csv(systems, sep=',', low_memory=False)
  #df_gnc = pd.read_csv(gnc, sep=',', low_memory=False)
  # TODO: Align the two files

  # Start building the unified data frame
  df = pd.DataFrame()
  headers = ['TIME', 'MET_SUBSECS', 'VEHICLE_STATE', ]
  pressures = ['ED01_PRESS', 'ED02_PRESS', 'ED03_PRESS', 'EGC_PRESS', 'N2_CTRL_VALVE_FEED_PRESS', 'PROP_TANKS_FEED_PRESS', 'N2_TANKS_PRESS', 'PROP_TANKS_PRESS', 'THRUSTERS_FEED_PRESS']
  temperatures = ['EGC_THROTTLE_MOTOR_TEMP', 'EA01_FUEL_FEED_TUBE_TEMP', 'ED01_FUEL_FEED_TUBE_TEMP', 'ED01_TEMP', 'ED02_TEMP', 'ED03_TEMP', 'EGC_FUEL_FEED_TUBE_TEMP', 'EGC_TEMP', 'MAIN_FUEL_FEED_TUBE_TEMP', 'N2_TANK1_TEMP', 'N2_TANK2_TEMP', 'PROP_TANK1_BOTTOM_TEMP', 'PROP_TANK1_TOP_TEMP', 'PROP_TANK2_BOTTOM_TEMP', 'PROP_TANK2_TOP_TEMP']
  currents = ['N2_ISO_VALVE_CURR', 'PROP_ISO_VALVE_CURR', 'PROP_LOAD_VALVE_CURR', 'ED01_CURR', 'ED02_CURR', 'ED03_CURR', 'EA01_CURR', 'EA02_CURR', 'EA03_CURR', 'EA04_CURR', 'EA05_CURR', 'EA06_CURR', 'EA07_CURR', 'EA08_CURR', 'EA09_CURR', 'EA10_CURR', 'EA11_CURR', 'EA12_CURR', 'EGC_THROTTLE_MOTOR_CURR']
  misc = ['LANDING_LEG_0', 'LANDING_LEG_1', 'LANDING_LEG_2', 'LAND_NOW_ABORT']
  controls = ['MGC_MASS_ESTIMATE', 'EGC_THROTTLE_POSITION']

  # Load the fields into the data frame
  for fields in [headers, pressures, temperatures, currents, misc, controls]:
    for field in fields:
      df[field] = df_systems[field]
  
  print(df)

  opath = os.path.join(args.output_dir, os.path.basename(systems).split('_')[0])
  name = os.path.basename(opath)
  save_data(opath, name, df)

  '''
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

      # Create the output directory and write out all the file types
      save_data(opath, name, df)
  '''

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
    # Perform the initial processing and cleanup of the raw data files
    p.map(process_file, [(args, filename) for filename in sorted(glob.glob(f'{args.input_dir}/**/*.csv', recursive=True))])

    # Create a single file with the columns and data structures that we need.
    #p.map(combine_file, [(args, path) for path in sorted(glob.glob(f'{args.output_dir}/*'))])
    

  except KeyboardInterrupt: 
    running = False
  except:
    traceback.print_exc()


if __name__ == '__main__':
  main()

