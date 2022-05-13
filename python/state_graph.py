#!/usr/bin/env python3
import sys
import os
import re
import time
import argparse
import traceback

parser = argparse.ArgumentParser(description='Parses a source file and extracts data to build a state transition graph.')
parser.add_argument('filename', help='The file to be parsed')
args = parser.parse_args()

try:
  states = []
  transitions = []
  with open(os.path.abspath(os.path.expanduser(args.filename)), 'r') as f:
    data = f.read().replace('\n', '')

    # Extract all the state transitions from the source file
    events = re.findall('addTransition.*?\((.*?(?=\)))', re.sub('\s+', ' ', data).strip().replace(' ', ''))
    for event in events:
      (a, b) = re.match(r'.*?::(.*),.*?::(.*)', event).group(1, 2)
      transitions.append('->'.join([a, b]))

    events = re.findall('addState.*?\((.*?(?=[\),]))', re.sub('\s+', ' ', data).strip().replace(' ', ''))
    for event in events:
      (a) = re.match(r'.*?::(.*)', event).group(1)
      states.append(a)

  with open('state_machine.gv', 'w') as f:
    f.write('digraph finite_state_machine {\n')
    f.write('  // Init State\n')
    f.write('  node [shape=point,label=""]\n')
    f.write('  INIT;\n\n')
    f.write('  // Valid States\n')
    f.write('  node [shape=circle];\n')
    for s in states:
      f.write(f'  {s}[label="{s}"]\n')
    f.write('\n  // Valid Transitions\n')
    f.write(f'  INIT->{states[0]} [label="Initialization"]\n')
    for t in transitions:
      f.write(f'  {t}\n')
    f.write('}')
  
except:
  traceback.print_exc()
  sys.exit(1)


