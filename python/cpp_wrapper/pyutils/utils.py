
import os
import subprocess

def run_cmd(cmd, working_dir=None, verbose=False, dryrun=False):
  if working_dir:
    working_dir = os.path.abspath(working_dir)
  if verbose:
    print '%sCmd: %s' % ('Working Dir: %s ' % (working_dir) if working_dir else '', cmd)

  if dryrun:
    return ''

  stdout = subprocess.check_output(cmd, cwd=working_dir, shell=True)

  if verbose:
    print stdout

  return stdout
