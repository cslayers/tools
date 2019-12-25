
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import sys

filename = sys.argv[1]
 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
 
x = []
y = []
z = []

f = open(filename, 'r') 
for line in f.readlines():
    line = line.strip()
    p = line.split(' ')
    x.append(float(p[0]))
    y.append(float(p[1]))
    z.append(float(p[2]))
    # print(p)


ax.scatter(x, y, z, c='k', marker='p', s=0.1)

plt.show()
 
 