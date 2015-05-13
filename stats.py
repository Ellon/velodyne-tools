#! /usr/bin/env python
import sys
data = sys.stdin.readlines()
intensities = [int(line.split()[3]) for line in data]
print((min(intensities), max(intensities)))
