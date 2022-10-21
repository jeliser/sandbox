#!/usr/bin/env python3
import argparse
import re
import os
import json
import tempfile
import traceback
import getpass
import urllib.parse

import requests
from requests.auth import HTTPBasicAuth


def main():
  parser = argparse.ArgumentParser(description='Publishes rows of data.')
  parser.add_argument('-j', '--jira', help='The JIRA URL', type=str, default='jira.atlassian.com')
  parser.add_argument('-p', '--prefix', help='The JIRA ticket prefix', type=str, default='JIRA')
  parser.add_argument('-u', '--user', help='The JIRA username', type=str, required=True)
  parser.add_argument('-t', '--txt', help='A text file with all the tickets in it.  A raw grep\'ed output is sufficient', type=str, required=True)
  parser.add_argument(      '--verbose-help', help='Prints out the verbose help menu', action='store_true', default=False)
  args = parser.parse_args()

  try:
    psword = getpass.getpass()
    step = 50
    data = []

    # Find all the tagged todo tickets and build the query
    tickets = sorted(set(re.findall(args.prefix + '-\d{1,6}', open(f'{args.txt}', 'r').read())))

    # Iterate over the buckets of tickets
    for i in range(0, len(tickets), step):
        url = 'https://{0}/rest/api/2/search?fields=status&jql=key in ({1})'.format(args.jira, ', '.join(tickets[i:i+step]))
        query = url.replace(" ", "%20")
 
        # Query JIRA for the ticket statues
        response = requests.get(query, auth=HTTPBasicAuth(args.user, psword))
        for issue in response.json()['issues']:
          data.append([issue['key'], issue['fields']['status']['name']])

    # Sort the listing and print out the results
    for issue in sorted(data, key=lambda x: (x[1], x[0])):
      print('{:<12} {:}'.format(*issue))

  except KeyboardInterrupt: 
    running = False
  except:
    traceback.print_exc()

if __name__ == '__main__':
  main()

