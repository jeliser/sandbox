#!/usr/bin/env python3

import argparse
import bitstruct
import traceback

def parse_data(data):
    # Define the format string for the data structure
    format_string = 'u2u6'

    try:
        # Unpack the data using the specified format string
        if bitstruct.calcsize(format_string) == len(data) * 8:
            unpacked_data = bitstruct.unpack(format_string, data)
            # Print the unpacked data
            print(unpacked_data)
        else:
            print(f'{len(data) * 8} != {bitstruct.calcsize(format_string)}')
    except bitstruct.Error as e:
        # Print the error message and traceback
        print(f"Error: {e}")
        traceback.print_exc()

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Data Parsing Script')
    parser.add_argument('data', type=str, help='Data to be parsed')
    args = parser.parse_args()

    # Call the parse_data function with the provided data
    parse_data(bytes.fromhex(args.data)) #args.data.encode())

if __name__ == '__main__':
    main()

