#!/usr/bin/python
# 

import os
import argparse
import yaml

DICTS  = 'message_dictionary'
NAME   = 'name'
FIELDS = 'fields'
ENUM   = 'enum'
TYPE   = 'type'
HASH   = 'message_id'
DESCRIPTION = 'description'


class FIELD(object):

  def __init__(self, field_dict):
    self.field = field_dict
    self.name  = self.field.get(NAME)
    self.type  = self.field.get(TYPE)
    self.enum  = True if ENUM in self.field else False
    self.description = self.field.get(DESCRIPTION)

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    return '%s' % (self.field)


class MESSAGE(object):

  def __init__(self, message_dict):
    self.message = message_dict
    self.name    = self.message.get(NAME)
    self.hash    = self.message.get(HASH)
    self.description = self.message.get(DESCRIPTION)

    self.fields  = []
    for field in self.message[FIELDS]:
      self.fields.append(FIELD(field))

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    return 'Message: %s\n Fields: %s' % (self.message, self.fields)

  ### Public functions to access underlying converted data
  def get_name(self):
    return self.name

  def get_hash(self):
    return self.hash


class ICD(object):

  def __init__(self, filename):
    # TODO: This should support multiple filenames
    self.messages = []
    self.__load(filename)

  def get_messages(self):
    return self.messages


  # Making these private methods
  def __load(self, filename):
    with open(filename, 'r') as f:
      try:
        self.yaml = yaml.load(f, Loader=yaml.FullLoader)
        # NOTE: This should probably validate the YAML file while it's parsing it, or there should be a validation script to check the ICDS message file before running this script
        for message in self.yaml[DICTS]:
          self.messages.append(MESSAGE(message))
      except yaml.YAMLError as exc:
        print(exc)

  def __str__(self):
    return '%s' % (str(self.messages))



if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Generate message from template files')
  parser.add_argument('-v', '--verbose', help='Verbose output', action='store_true', default=False)
  parser.add_argument(      '--dryrun', help='Dryrun the commands', action='store_true', default=False)
  
  parser.add_argument('-i', '--icd', help='The ICD YAML file to load', required=True)

  args = parser.parse_args() 

  icds = ICD(args.icd)
  print(icds.get_messages())

