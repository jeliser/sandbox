#!/usr/bin/env python
import os
import sys
import argparse
import traceback
import csv
import glob
import yaml
import multiprocessing
import time
import datetime
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
  # Unpack the parameters
  (args, name) = tup

  try:
    # Scenarios that should be ignored
    ignore = ['Purge', 'LTF01', 'LTF03', 'MEG']
    if any(filter(lambda i: i in name, ignore)):
      return
 
    # We only want to process a subset of files 
    if any(filter(lambda n: n in name, ['Systems.csv', 'GNCA.csv'])):
      # Read the data into a pandas data frame and remove all the duplicate lines to only leave the latest updated row.
      df = pd.read_csv(name, sep=',', low_memory=False)
      # Automatically remove the system name from the column headers
      df.columns = [ col.split(':')[-1] for col in df.columns ]
 
      opath = os.path.join(args.output_dir, os.path.basename(name).split('_')[0])
      print(f'Filtering: {name.split(args.input_dir)[-1]} to {opath}')
      
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

        # Fix the TIME column to a ISO standard representation
        #for frame in [df_systems, df_gnc]:
        #  frame['TIME'] = frame['TIME'].apply(lambda f: str(datetime.datetime.strptime(f, "%Y-%j %H:%M:%S.%f").isoformat('T')))

        # Extract the subset of data from the original data frame using the filtered indexes
        save_data(opath, name, df.iloc[idx])
  except:
    print(f'Failed filtering {name}')
    traceback.print_exc()

'''
Method for processing the files into better data structures
'''
def combine_file(tup):
  (args, path) = tup

  try:
    print(f'Processing: {os.path.basename(path)}')

    # Grab the files we are going to process
    files = sorted(glob.glob(f'{path}/*.csv'))
    systems = list(filter(lambda f: 'System' in f, files))[-1]
    gnc     = list(filter(lambda f: 'GNC' in f, files))[-1]
 
    # Align data between the Systems file and the GNC file
    df_systems = pd.read_csv(systems, sep=',', low_memory=False)
    df_gnc = pd.read_csv(gnc, sep=',', low_memory=False)

    # Align the data rows
    # Common column names between the two files SC_ACC_0, VEHICLE_STATE
    # Secondarily TIME can be used if those fields are not available
    # TODO: It's possible that other files might have transitive relationships that we can use to get a perfect match:  A(x)->B(x, y)->C(y, z)
 
    # Start off with just using the TIME and finding the closest match.
    for frame in [df_systems, df_gnc]:
      frame['TIME'] = frame['TIME'].apply(lambda t: datetime.datetime.strptime(t, "%Y-%j %H:%M:%S.%f"))

    # Make sure the number of rows is the same between the files (removes the NaN at the end)
    min_rows = min(len(df_systems.index), len(df_gnc.index))
    df_systems = df_systems[:][0:min_rows]
    df_gnc = df_gnc[:][0:min_rows]

    # TODO: Not going to worry about this time alignment at the moment.  We can remove the problematic lines in the source files
    '''
    # Computed the time difference between the two dataset
    times = pd.DataFrame()
    times['SYSTEMS'] = df_systems['TIME']
    times['GNC'] = df_gnc['TIME']
    times['DIFF'] = times.apply(lambda r: (r[0] - r[1]).total_seconds(), axis=1)

    print(path)
    print(times.sort_values(by=['DIFF']))
    print(df_systems['MET_SUBSECS'].diff().sort_values())
    print('\n\n')
    '''

    #print(type(df_systems['TIME'][0]))
    #print(df_systems['TIME'].sub(df_gnc['TIME'], axis = 0))
    #print(df_systems['TIME'][0], df_gnc['TIME'][0], (df_systems['TIME'][0] - df_gnc['TIME'][0]).total_seconds())

    # Load the UNIX time into the output data frame
    # TODO: Coerce the timestamp to be clean 10Hz steps
    df = pd.DataFrame()
    df.insert(0, 'UNIX_TIME', df_systems['TIME'].apply(lambda t: time.mktime(t.timetuple())+t.microsecond*1E-6))
    df.insert(1, 'UNIX_MET', df['UNIX_TIME'].diff().fillna(0).apply(lambda t: float('{:0.3f}'.format(t))))

    for frame in [df_systems, df_gnc]:
      frame['TIME'] = frame['TIME'].apply(lambda f: f.isoformat('T'))
 
    # Start building the unified data frame
    headers = ['TIME', 'MET_SUBSECS']
    pressures = ['ED01_PRESS', 'ED02_PRESS', 'ED03_PRESS', 'EGC_PRESS', 'N2_CTRL_VALVE_FEED_PRESS', 'N2_PROP_TANKS_FEED_PRESS', 'N2_TANKS_PRESS', 'PROP_TANKS_PRESS', 'THRUSTERS_FEED_PRESS']
    temperatures = ['EGC_THROTTLE_MOTOR_TEMP', 'EA01_FUEL_FEED_TUBE_TEMP', 'ED01_FUEL_FEED_TUBE_TEMP', 'ED01_TEMP', 'ED02_TEMP', 'ED03_TEMP', 'EGC_FUEL_FEED_TUBE_TEMP', 'EGC_TEMP', 'MAIN_FUEL_FEED_TUBE_TEMP', 'N2_TANK1_TEMP', 'N2_TANK2_TEMP', 'PROP_TANK1_BOTTOM_TEMP', 'PROP_TANK1_TOP_TEMP', 'PROP_TANK2_BOTTOM_TEMP', 'PROP_TANK2_TOP_TEMP']
    currents = ['N2_ISO_VALVE_CURR', 'PROP_ISO_VALVE_CURR', 'PROP_LOAD_VALVE_CURR', 'ED01_CURR', 'ED02_CURR', 'ED03_CURR', 'EA01_CURR', 'EA02_CURR', 'EA03_CURR', 'EA04_CURR', 'EA05_CURR', 'EA06_CURR', 'EA07_CURR', 'EA08_CURR', 'EA09_CURR', 'EA10_CURR', 'EA11_CURR', 'EA12_CURR', 'EGC_THROTTLE_MOTOR_CURR']
    misc = ['LANDING_LEG_0', 'LANDING_LEG_1', 'LANDING_LEG_2', 'LAND_NOW_ABORT', 'VEHICLE_STATE']
    controls = ['MGC_MASS_ESTIMATE', 'EGC_THROTTLE_POSITION']
 
    # Load the system fields into the data frame
    for fields in [headers, pressures, temperatures, currents, misc, controls]:
      for field in fields:
        df[field] = df_systems[field]

    navigation = ['SC_POS_0', 'SC_POS_1', 'SC_POS_2', 'SC_VEL_0', 'SC_VEL_1', 'SC_VEL_2', 'SC_ACC_0', 'SC_ACC_1', 'SC_ACC_2', 'SC_ALT']

    # Load the system fields into the data frame
    for fields in [navigation]:
      for field in fields:
        df[field] = df_gnc[field]


    # Check for the GNCT and include it if it's available
    if any(filter(lambda f: 'GNCT' in f, df_gnc.columns)):
      truth = ['GNCT_XCM_0', 'GNCT_XCM_1', 'GNCT_XCM_2', 'GNCT_XCM_3', 'GNCT_XCM_4', 'GNCT_XCM_5', 'GNCT_XCM_6', 'GNCT_XCM_7', 'GNCT_XCM_8', 'GNCT_XCM_9']
      for fields in [truth]:
        for field in fields:
          df[field] = df_gnc[field]

    # Wrap the data to the unified output file
    opath = os.path.join(args.output_dir, os.path.basename(systems).split('_')[0])
    name = os.path.basename(opath)
    save_data(opath, name, df)
 
  except:
    print(f'Failed processing {path}')
    traceback.print_exc()


'''
Main entrypoint
'''
def main():
  parser = argparse.ArgumentParser(description='Publishes rows of data.')
  parser.add_argument('-i', '--input-dir', help='The directory with the raw CSV files to be processed', required=True)
  parser.add_argument('-o', '--output-dir', help='The directory to place the converted CSV data', required=True)
  parser.add_argument('-j', '--pool', help='The number of threads to put in the thread pool (default: 4)', default=4, type=int)
  parser.add_argument(      '--skip-filtering', help='Skip the filtering process', action='store_true', default=False)
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
    if not args.skip_filtering:
      # Filter the initial processing and cleanup of the raw data files
      p.map(process_file, [(args, filename) for filename in sorted(glob.glob(f'{args.input_dir}/**/*.csv', recursive=True))])

    # Create a single file with the columns and data structures that we need.
    p.map(combine_file, [(args, path) for path in sorted(glob.glob(f'{args.output_dir}/*'))])
  except KeyboardInterrupt: 
    running = False
  except:
    traceback.print_exc()


if __name__ == '__main__':
  main()

