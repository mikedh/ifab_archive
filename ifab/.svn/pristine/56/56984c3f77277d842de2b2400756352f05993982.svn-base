from scipy.spatial import ckdtree as kd
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy


d = 10*numpy.random.random((10,3))
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.plot(plotPts[:,0], plotPts[:,1], plotPts[:,2])
ax.scatter(d[:,0], d[:,1], d[:,2])


t = kd.cKDTree(d)
print t.query([0,0,0], 2)

raw_input('hi')
print numpy.sum(d**2, axis=1)
plt.show()
