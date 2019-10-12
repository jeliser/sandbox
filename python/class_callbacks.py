#!/usr/bin/env python3

import sys
import os
import functools

from MessageInterface import MessageInterface

from data_model.raw.double import Double as Double
from data_model.duplicate.namespace.double import Double as DDouble
#import data_model.raw.double.Double


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

# Takes a string and 
def str_to_class(class_name):
    try:
        return functools.reduce(getattr, class_name.split("."), sys.modules[__name__])
    except:
        return None

# Show that we can take a class name as a string and find a matching class implementation
print(str_to_class("MessageInterface"))
print(str_to_class("Color"))
print(str_to_class("Float"))
print(str_to_class("Double"))
print(str_to_class("DDouble"))
print(str_to_class("data_model.raw.double.Double"))
print(str_to_class("fake_class"))
print(DDouble)

# Create instances of the class
m = MessageInterface()
c = Color("red")
f = Float("my_value")
dd = DDouble("my_double")



