#!/usr/bin/env python3

import sys
import os
import functools
from multimethod import overload, isa, multimeta

from MessageInterface import MessageInterface

from data_model.raw.double import Double as Double
from data_model.duplicate.namespace.double import Double as DDouble


###################################
# Simple inline data model ... the other types are imported

class Color(MessageInterface):
    def __init__(self, topic = ""):
        self.set_topic(topic)
        self.color = ""

    def get_color(self):
        return self.color

    def set_color(self, color):
        self.color = color


class Float(MessageInterface):
    def __init__(self, topic = ""):
        self.set_topic(topic)
        self.double = 0.0

    def get_double(self):
        return self.double

    def set_double(self, double):
        self.double = double


# Takes a string and looks for a matching class in the module space
def str_to_class(class_name):
    try:
        return functools.reduce(getattr, class_name.split("."), sys.modules[__name__])
    except:
        return None

###################################

# Show that we can take a class name as a string and find a matching class implementation
class_names = ["MessageInterface", "Color", "Float", "Double", "DDouble"]

lookup = dict()
for class_name in class_names:
    lookup[class_name] = str_to_class(class_name)

if not all(lookup.values()):
    print('Failed to lookup a class typen:\n{}'.format(lookup));

# Create instances of the classes that we can use in the callbacks
m = MessageInterface()
c = Color("red")
f = Float("my_value")
d = Double("my_double")
dd = DDouble("my_duplicate_double")

###################################
## Example of using overloads outside of a class definition

## All the overloaded functions
@overload
def callback(data: isa(MessageInterface)):
    print('{:16} -- {}'.format('MessageInterface', data))

@overload
def callback(data: isa(Float)):
    print('{:16} -- {}'.format('Float', data))

@overload
def callback(data: isa(Color)):
    print('{:16} -- {}'.format('Color', data))

# This will trigger the specific callbacks
print('Non-class methods:')
callback(f)
callback(c)
# This will trigger the catch-all callback
callback(d)
callback(dd)
print('')

###################################
## Example of using overloads inside of a class definition

class DataScienceApplication(metaclass=multimeta):
    def on_data(self, data: Float):
        print('{:16} -- {}'.format('Float', data))

    def on_data(self, data: Double):
        print('{:16} -- {}'.format('Double', data))

    def on_data(self, data: DDouble):
        print('{:16} -- {}'.format('DDouble', data))

    def on_data(self, data: MessageInterface):
        print('{:16} -- {}'.format('MessageInterface', data))


app = DataScienceApplication()


print('Class methods:')
app.on_data(f)
app.on_data(c)
app.on_data(d)
app.on_data(dd)

