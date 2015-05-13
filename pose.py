#! /usr/bin/env python
import sys
lines = sys.stdin.readlines()
# lines = f = open('velodyneShot.pos.0000').readlines()

parser = lambda label: map(float, [line for line in lines \
    if line.startswith(label)][0].strip().split()[2:])

sensor_to_main = parser('sensorToMain = ')
main_to_origin = parser('mainToOrigin = ')

# print((sensor_to_main, main_to_origin))

from math import sin, cos
def matrix(yaw, pitch, roll, x, y, z):
    # from pom-genom/libeuler/pomEuler.c:287 (pomWriteSensorPos)
    # euler.{yaw,pitch,roll,x,y,z}
    ca, sa = cos(yaw),   sin(yaw)
    cb, sb = cos(pitch), sin(pitch)
    cg, sg = cos(roll),  sin(roll)
    return [[ ca*cb, ca*sb*sg - sa*cg, ca*sb*cg + sa*sg, x],
            [ sa*cb, sa*sb*sg + ca*cg, sa*sb*cg - ca*sg, y],
            [ -sb, cb*sg, cb*cg, z],
            [ 0.0, 0.0, 0.0, 1.0]]

import numpy as np
sensor_to_origin = np.dot(matrix(*sensor_to_main), matrix(*main_to_origin))
print(sensor_to_origin)
