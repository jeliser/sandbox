#!/usr/bin/env python3
# coding: utf-8

import os
import subprocess

def print_results(output, err):
  # TODO: [S1-163] This should probably do something on error
  if any(output):
    print(output.decode("utf-8"))
  if any(err):
    print(err.decode("utf-8"))
    sys.exit(1)

def run_cmd(cmd, cwd=None, verbose=False, dryrun=False):
  if cwd:
    cwd = os.path.abspath(cwd)

  if dryrun:
    return ''

  p = subprocess.run(cmd, cwd=cwd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

  if verbose:
    print_results(p.stdout, p.stderr)

  return p.stdout.decode("utf-8") + p.stderr.decode("utf-8")
import os

from flask import Flask, render_template, render_template_string, request, Response, jsonify

import http
import json
import urllib.request, urllib.parse

import re
import cpp_wrapper.pyutils.utils

app = Flask(__name__, template_folder="templates")

@app.route('/execute')
def execute():
  cmd = urllib.parse.unquote(request.args.get('cmd', ''))
  ret = run_cmd(cmd)

  return jsonify(
    cmd=cmd,
    ret=ret
  )


if __name__ == "__main__":
  app.run(debug=True, use_reloader=True)
