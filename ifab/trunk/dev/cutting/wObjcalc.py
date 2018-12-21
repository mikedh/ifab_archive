import numpy
import transformations
from matplotlib import pyplot as plt



def vectorAngle(v1, v2):
    return numpy.arccos((numpy.dot(v1,v2)/(numpy.linalg.norm(v1) * numpy.linalg.norm(v2))))


def unitVector(points):
    if len(numpy.shape(points)) == 1: 
        return points / numpy.linalg.norm(points)
    else: 
        return points / (numpy.reshape(numpy.sqrt(numpy.sum(points**2, axis=1)), (numpy.shape(points)[0],1)))


def points2vector(points):
    points = unitVector(points)
    return unitVector(numpy.average(points, axis=0))

    


scount = 2

N = 25.4

points = numpy.random.random((scount,5,2))
#points[0] = [[874.4,930.0], [918.7, 986.4], [949.8, 1025.4], [970.1, 1052.8], [995.7, 1085.2]]
#points[1] = [[778.2, 1340.7], [824.1, 1325.3],[ 891.1, 1300.6], [926.5, 1288.4], [952.1, 1279.0]]
points[0] = [[948.8,959.5],[984.2,1003.8],[1028.0,1061.0],[1055.9,1096.1],[1076.5,1122.6]]
points[1] = [[847.9,1195.1],[898.0,1190.4],[953.0,1186.7],[997.9,1182.7],[0,0]]


mc = numpy.zeros((scount, 2))

for i, pset in enumerate(points):
    psi = numpy.array((numpy.int_(numpy.sum(pset, axis=1)) <> 0)).flatten()
    x = pset[:,0].flatten()
    x = x[psi]
    y = pset[:,1] .flatten()
    y = y[psi]

    A = numpy.vstack([x, numpy.ones(len(x))]).T
    mc[i] = numpy.linalg.lstsq(A, y)[0]
    
    
    mc[i][1] += N / numpy.cos(numpy.arctan(mc[i][0]))
    plt.plot(x, mc[i][0]*x + mc[i][1])


    plt.plot(x, y, 'o', markersize=5)
    

print mc
# y = mx + c = m1x + c1
m = mc[0][0]; c = mc[0][1]
m1 = mc[1][0]; c1 = mc[1][1]
 
xi = (c1-c)/(m-m1)
yi = m*xi + c
plt.plot(xi, yi, 'ko', markersize=10)

vec = numpy.append(points2vector(points[0]),0)

print 'rotation angle of wobj', (vectorAngle(points2vector(points[0]), [1,0]))
print 'rang m', numpy.arctan(m)
print 'calculated intersection', [xi, yi]

plt.show()
