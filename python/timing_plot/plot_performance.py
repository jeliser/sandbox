#!/usr/bin/env python3
import struct
import matplotlib.pyplot as plot
import numpy as np
import yaml
import argparse
import sys
import subprocess

def run_cmd(cmd):
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, err = p.communicate()
    return output.decode('utf-8')

parser = argparse.ArgumentParser(description='Generate message from template files')
parser.add_argument('filename', help='The performance file to be parsed')
args = parser.parse_args()

# Read in all the data from the sample file
with open(args.filename, 'r') as f:
    try:
        results = yaml.load(f)['results']

    except yaml.YAMLError as exc:
        print(exc)
        sys.exit(1)

'''
# Generate the subplots
h, p = plot.subplots(len(results), sharex=True, sharey=True)
i = 0
for result in results:
    # Convert to seconds, and plot the step function
    p[i].plot(range(len(result['results'])), result['results'])
    #plot.yticks([1, 2], ('Start', 'Stop'))
    #plot.ylim((0, 3))
    i += 1
'''

# Generate the subplots
ax = plot.axes()

## Build a stem and leaf plot
data = []
names = []
max_name_len = 20
print('              Object Type         min (ns)   mean (ns)   max (ns)      std')
print('--------------------------------------------------------------------------------')
for result in results:
    values = list(map(int, result['values']))
    data.append(values)
    names.append((result['name'][:max_name_len] + '..') if len(result['name']) > max_name_len else result['name'])
    print('%30s  %8d   %8d    %8d     %f' % (result['name'], np.min(values), np.mean(values), np.max(values), np.std(values)))
plot.boxplot(data)
ax.set_xticklabels(names)
plot.xticks(rotation=75)
plot.subplots_adjust(bottom=0.3)
plot.xlabel('Data Model Object Name')
plot.ylabel('Elapsed Time (nsec)')
plot.title('Testing Data Model Object Lifecycle - C++\n(instantiate, populate, serialize, and deserialize)\n%d iterations on a %s\n' % (len(results[0]['values']), run_cmd('uname -rsp')))
#plot.title('Testing Data Model Object Lifecycle - C++\n(instantiate, populate, serialize, and deserialize)\n%d iterations on a %s\n' % (len(results[0]['values']), 'Thomson imx6'))

'''
## Build a line graph
for result in results:
    plot.plot(range(len(result['results'])), list(map(int, result['results'])), label=result['name'])
plot.xlabel('Object Iteration')
plot.ylabel('Object Creation Time (nsec)')
plot.legend(loc='upper right')
'''

plot.show()
