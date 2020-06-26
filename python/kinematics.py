#!/usr/bin/env python
import argparse
import numpy
import math

def main():
    parser = argparse.ArgumentParser(description='Publishes rows of data.')
    args = parser.parse_args()

    numpy.set_printoptions(precision=3, floatmode='fixed')

    ## Assumes the steer point and contact point are the same position

    W = 1.5
    L = 5.0

    Vx = 12.0
    Vy = 10.0
    w  = math.radians(math.pi / 2)

    #      [ x,  y]
    # r1 = [ L,  W]
    # r2 = [ L, -W]
    # r3 = [-L,  W]
    # r4 = [-L, -W]

    Vs = numpy.transpose(numpy.array([[Vx, Vy, w]]))
    Hs = numpy.array([[1.0, 0.0, -W],
                      [0.0, 1.0,  L],
                      [1.0, 0.0,  W],
                      [0.0, 1.0,  L],
                      [1.0, 0.0, -W],
                      [0.0, 1.0, -L],
                      [1.0, 0.0,  W],
                      [0.0, 1.0, -L]])

    v = numpy.dot(Hs, Vs)
    IVs = numpy.dot(numpy.dot(numpy.linalg.inv(numpy.dot(numpy.transpose(Hs), Hs)), numpy.transpose(Hs)), v)

    #print('Vs:\n{}'.format(Vs))
    #print('Hs:\n{}'.format(Hs))
    #print('v:\n{}'.format(v))
    #print('IVs:\n{}'.format(IVs))

    print('Vs:  {}'.format(numpy.transpose(Vs)))
    print('IVs: {}'.format(numpy.transpose(IVs)))




if __name__ == '__main__':
  main()

