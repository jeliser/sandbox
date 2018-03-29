#!/usr/bin/python
# 

import os
import argparse
import jinja2 as jinja

import parser.ICD as ICD


def write(fullpath, text):
  if not os.path.exists(os.path.dirname(fullpath)):
    os.makedirs(os.path.dirname(fullpath))

  f = open(fullpath, 'w')
  f.write(text)
  f.close()


def main():
  parser = argparse.ArgumentParser(description='Generate message from template files')
  parser.add_argument('-v', '--verbose', help='Verbose output', action='store_true', default=False)
  parser.add_argument(      '--dryrun', help='Dryrun the commands', action='store_true', default=False)
  
  parser.add_argument('-t', '--template-dir', help='Set the Jinja language specific template directory', default='./templates')
  parser.add_argument('-i', '--icd-dir', help='Set the ICD YAML file directory', default='./icds')
  parser.add_argument('-o', '--out-dir', help='Set the generated output directory', default='./generated')

  args = parser.parse_args() 

  # Setup the Jinja environment from the directory that this script is executed
  env = jinja.Environment(loader=jinja.FileSystemLoader(os.path.dirname(os.path.abspath(__file__))), trim_blocks=True)

  # Load some sample YAML ICD files
  icds = ICD.ICD(os.path.join(args.icd_dir, 'FileManagerMessages.yml'))

  # Generate the C++ output files
  cpp_path = os.path.join(args.out_dir, 'cpp')
  for msg in icds.get_messages():
    write(os.path.join(cpp_path, msg.get_name() + ".cpp"), env.get_template(args.template_dir+'/cpp/Message.cpp').render(msg=msg))


if __name__ == "__main__":
  main()

