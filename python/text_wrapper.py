#!/usr/bin/python

import textwrap

text  = 'hello this is a really long line and should be split up into multiple rows'

for t in textwrap.wrap(text, 25):
    print t


