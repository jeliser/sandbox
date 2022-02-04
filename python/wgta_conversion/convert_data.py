#!/usr/bin/env python
import os
import sys
import argparse
import traceback
import csv
import glob


def main():
    parser = argparse.ArgumentParser(description='Publishes rows of data.')
    parser.add_argument('-i', '--input-dir', help='The directory with the raw CSV files to be processed', required=True)
    parser.add_argument('-o', '--output-dir', help='The directory to place the converted CSV data', required=True)
    parser.add_argument(      '--verbose-help', help='Prints out the verbose help menu', action='store_true', default=False)
    args = parser.parse_args()

    args.input_dir = os.path.abspath(args.input_dir)
    args.output_dir = os.path.abspath(args.output_dir)

    # Check if we're just using the verbose output
    if args.verbose_help:
        print('More help')
        sys.exit()

    try:
      for name in glob.glob(f'{args.input_dir}/*/*.csv'):
        with open(name, 'r') as csvfile:
          print(name)
          reader = csv.reader(csvfile, delimiter=',')
          header = [ h.split(':')[-1] for h in next(reader, None) ]
          print(header)
          for row in reader:
            print(row)
            break
      pass
    except KeyboardInterrupt: 
      running = False
    except:
      traceback.print_exc()


if __name__ == '__main__':
  main()

