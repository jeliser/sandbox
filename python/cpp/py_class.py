'''py_class.py - Python source designed to demonstrate the use of python embedding'''
import abc
import pyaml
import asyncio

class MultiplyInterface(abc.ABC):
    @abc.abstractmethod
    def multiply(self):
        raise NotImplemented

    #@abc.abstractmethod
    def again(self):
        raise NotImplemented


class Multiply(MultiplyInterface):
    def __init__(self): 
        self.a = 6 
        self.b = 5 
    
    def multiply(self):
        c = self.a*self.b
        print('The result of', self.a, 'x', self.b, ':', c)
        return c
    
    def multiply2(self, a, b):
        c = a*b
        print('The result of', a, 'x', b, ':', c)
        return c                  


#m = Multiply()
#m.multiply()
