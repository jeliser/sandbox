#!/usr/bin/env python3

from MessageInterface import MessageInterface

class Double(MessageInterface):
    def __init__(self, topic = ""):
        self.set_topic(topic)
        self.double = 0.0

    def get_double(self):
        return self.double

    def set_double(self, double):
        self.double = double

