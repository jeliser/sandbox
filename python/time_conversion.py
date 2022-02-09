#!/usr/bin/env python3

import datetime


TIME = '2012-229 16:49:48.137'


date_time = datetime.datetime.strptime(TIME, "%Y-%j %H:%M:%S.%f")
print(date_time)
