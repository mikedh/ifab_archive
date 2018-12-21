import numpy, json

#import matplotlib.pyplot as plt


def transform(tMat, points):
    if numpy.shape(tMat) <> (4,4): return False
    if numpy.shape(points)[1] <> 3: return False

    res = numpy.zeros((len(points), 3))
    for i in xrange(len(points)):
        res[i] = (numpy.dot(tMat, numpy.append(points[i],1)))[0:3]
    return res


#x, y, theta are all floats, rotCenter is [x,y]
def transformationMatrix(x, y, theta, rotCenter=[0,0]):
    tr_back = numpy.float64(numpy.eye(4))
    tr_back[0,3] = -rotCenter[0]
    tr_back[1,3] = -rotCenter[1]
    
    tr_for = numpy.float64(numpy.eye(4))
    tr_for[0,3] = rotCenter[0]
    tr_for[1,3] = rotCenter[1]

    #adds the translation in X and Y
    tr_mat = numpy.float64(numpy.eye(4))
    tr_mat[0,3] = x
    tr_mat[1,3] = y
    
    rot_mat = numpy.float64(numpy.eye(4));
    rot_mat[0,0] = numpy.cos(theta)
    rot_mat[0,1] = -numpy.sin(theta)
    rot_mat[1,0] = numpy.sin(theta) 
    rot_mat[1,1] = numpy.cos(theta) 

    transformation_matrix = numpy.dot(rot_mat,tr_back);
    transformation_matrix = numpy.dot(tr_for, transformation_matrix);
    transformation_matrix = numpy.dot(tr_mat, transformation_matrix);
    return (transformation_matrix)


def finalTransform(tMatNew, tMat0):
    return numpy.dot(tMatNew, tMat0)


#a possible error function that finds the sum distance squared of all points to the origin
def dumbError(x, y, theta, points, rCenter=[0,0]):
    err = 0
    tNew = transformationMatrix(x, y, theta, rCenter)
    pNew = transform(tNew, points)
    for i in pNew:
        err += numpy.sum(i[0:2]**2)
    return err

if __name__ == '__main__':
    
    j = json.load(open('flatWelds.json', 'r'))
    
    #center of the points, so we can rotate around it. You could not do this
    rCenter = numpy.average(j['weldPoints'], axis=0)[0:2]

    #find a new transformation matrix based on x, y and theta
    tNew = transformationMatrix(10, 10, numpy.radians(45), rCenter)

    #new set of points based on the transformation matrix we generated
    pNew = transform(tNew, j['weldPoints'])


    print 'no transform error', dumbError(0,0,0, j['weldPoints'])
    print '-2000 y transform error', dumbError(0,-2000,0, j['weldPoints'])

    #plt.plot(pNew[:,0], pNew[:,1], 'o')
    #plt.show()
    
   

    print 'now into the json file goes',  finalTransform(tNew, j['tMat0'])

    
    
