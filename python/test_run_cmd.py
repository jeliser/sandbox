#!/usr/bin/python

import argparse

import pyutils.utils as utils


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Test application for the run command library')
  parser.add_argument('-v', '--verbose', help='Verbose output', action='store_true', default=False)
  parser.add_argument(      '--dryrun', help='Dryrun the commands', action='store_true', default=False)
  
  parser.add_argument('-c', '--cmd', help='Command to execute', required=True)
  parser.add_argument(      '--working-dir', help='Set the working directory', default=None)

  args = parser.parse_args() 

  utils.run_cmd(cmd=args.cmd, working_dir=args.working_dir, verbose=args.verbose, dryrun=args.dryrun)
