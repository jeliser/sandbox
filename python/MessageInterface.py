#!/usr/bin/env python3

class MessageInterface():
    def __init__(self):
        self.topic = ""

    def get_topic(self):
        return self.topic

    def set_topic(self, topic):
        self.topic = topic

