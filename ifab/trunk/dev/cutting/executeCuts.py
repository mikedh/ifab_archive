import numpy, json, time, sys
import transformations
import robot
from copy import deepcopy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculateIngress(points, baseIngress=[1,2,2,2,80,2], clearAngle=25 ):
    if len(numpy.shape(points)) <> 1: cart = numpy.average(points, axis=0)
    else: cart = points
    ang = numpy.degrees(numpy.arctan2(cart[1], cart[0]))
    if numpy.abs(ang) > 90: ang = (180-numpy.abs(ang))*numpy.sign(ang)
    baseIngress[0] = ang + numpy.sign(ang)*clearAngle
    
    return baseIngress

def unitVector(points):
    if len(numpy.shape(points)) == 1: 
        return points / numpy.linalg.norm(points)
    else: 
        return points / (numpy.reshape(numpy.sqrt(numpy.sum(points**2, axis=1)), (numpy.shape(points)[0],1)))


#compares array dimensions to dimensions tuple
#wildcard is -1, so an (n, 3) array called pts would be checked:
# dimCheck(pts, (-1,3))
def dimCheck(array, dim):
    sh = numpy.array(numpy.shape(array)); dim = numpy.array(dim)
    if len(sh) == len(dim):
        dc = dim <> -1
        if numpy.all(sh[[dc]] == dim[[dc]]): return True
        else: return False
    else:
        return False

def transform(points, T, onePad=False):

    #print 'T\n\n', T, '\n\npoints\n\n', points
    if len(numpy.shape(points)) == 1:
        return numpy.dot(numpy.append(points, numpy.zeros(int(onePad))),T)
    if onePad: 
        points = numpy.hstack((points, numpy.ones((numpy.shape(points)[0], 1))))
        return (numpy.dot(T, numpy.array(points).transpose()).transpose())[:,0:(numpy.shape(T)[1]-1)]
    return (numpy.dot(T, numpy.array(points).transpose()).transpose())

def rQuat(quats):
    for i in xrange(len(quats)):
        quats[i] = transformations.quaternion_multiply(quats[i], transformations.quaternion_about_axis(numpy.pi, [1,0,0]))
    return quats


def vectorAngle(v1, v2):
    return numpy.arccos((numpy.dot(v1,v2)/(numpy.linalg.norm(v1) * numpy.linalg.norm(v2))))


def q2vec(q):
    return numpy.dot(transformations.quaternion_matrix(q), [0,0,1,1])[0:3]


def thresh(value, t):
    q = numpy.abs(value)
    s = numpy.sign(value)
    if q > t: return t*s
    return q*s


def lamePath(dist, ang, side=50.8, r=3, oset=10, compAng=numpy.radians(0)):
    
    q = numpy.array([[0.,0., -0.71, 0.71],[0,0,1,0],[0.,0., 0.71, 0.71]])
    if numpy.round(ang%3.141,2) > .01: d = side / numpy.tan(ang)
    else: d = 0
    do = oset*numpy.cos(ang)
    zo = oset*numpy.sin(ang)
    s = numpy.array([[-do,side/2,-zo], [d+do,side/2,side+zo]])
    s1 = numpy.array([[d+do,-side/2,zo+side], [-do,-side/2,-zo]])
    top = [[d, ((side/2)+oset), side] ,[d, -((side/2)+oset) , side]]

    topAng=(numpy.pi/2)-ang
    botAng= -topAng
    
    topAng = thresh(topAng, numpy.radians(50))
    botAng = thresh(botAng, numpy.radians(50))


    qBot = transformations.quaternion_multiply(q[1], transformations.quaternion_about_axis(botAng, [0,1,0]))

    print '\n\n\QBOT', qBot

    angVec = [numpy.cos(ang), 0, numpy.sin(ang)]

    q[0] = transformations.quaternion_multiply(q[0], transformations.quaternion_about_axis(-compAng, angVec))
    q[1] = transformations.quaternion_multiply(q[1], transformations.quaternion_about_axis(topAng, [0,1,0]))
    q[2] = transformations.quaternion_multiply(q[2], transformations.quaternion_about_axis(compAng, angVec))

    q = unitVector(q)
    
    s = numpy.hstack((s, [q[0], q[0]]))
    s2 = numpy.hstack((s1, [q[2], q[2]]))
    top = numpy.hstack((top, [q[1], q[1]]))

    top = numpy.array([s,top,s2])

    numpy.set_printoptions(precision=3, suppress=True)
    top[:,:,0] -= numpy.min(top[:,:,0]) + dist
    
    bf = 1.375
    bottom = [[-dist+(bf*do), -((side/2)+oset), side] ,[-dist+(bf*do), (side/2)+oset , side]]
    

    bottom = numpy.hstack((bottom, [qBot,qBot]))
    
    return (top, bottom)



#takes vec, and sees if its pointing in generalDirection. if it is, or if the direction/vec are perpendicular, it returns vec.
#if it is pointing in the opposite direction, it returns -vec
def vecDir(vec, generalDirection):
    uvec = vec / numpy.linalg.norm(vec)
    generalDirection = generalDirection / numpy.linalg.norm(generalDirection)
    d = numpy.dot(uvec, generalDirection)
    if d == 0: return vec
    else: return numpy.sign(d)*numpy.array(vec)

def dist(points):
    points = numpy.array(points)
    v = points[0][0:3] - points[1][0:3]
    return numpy.sqrt(numpy.sum(v**2))

def rad(points, desR=3):
    
    vm = numpy.cross(numpy.dot(transformations.quaternion_matrix(points[0][3:7]), [0,0,1,1])[0:3], [1,0,0])
    vi = numpy.cross(numpy.dot(transformations.quaternion_matrix(points[1][3:7]), [0,0,1,1])[0:3], [1,0,0])
    vm = vecDir(vm, (points[0][0:3] - points[1][0:3]))
    vi = vecDir(vi, (points[1][0:3] - points[0][0:3]))
                
    a = vectorAngle(vm, vi)%numpy.pi
    
    if int(a) == 0: return points
    else:
        
        r = (dist(points)/2)/numpy.sin(a/2)
        #print 'radius', r, vi, vm

        points[0][0:3] -= vm*(r-desR)
        points[1][0:3] -= vi*(r-desR)


    return points
        

def reduceRadius(points):

    #print points
    for i in xrange(len(points)-1):
        points[i:i+2] = rad(points[i:i+2])
    

    return points

def tQuatF(quat, qR):
    for i in xrange(len(quat)):
        quat[i] = transformations.quaternion_multiply(quat[i], qR)
    return quat





    





def readinMain():    
    ing = [62.7,30.6,45.2,79.5,-42.2,-102.9]
    eg = [18,42,58,89,89,-58.4]
    veg = [18,0,58,89,89,-58.4]

    cutS = (30,200)
    rON = '-r' in sys.argv
    pON = '-p' in sys.argv
    

 
    topQuat = [[0.,0., -0.71, 0.71], [0.,0., -0.71, 0.71], [0,0,1,0], [0,0,1,0], [0.,0., 0.71, 0.71], [0.,0., 0.71, 0.71]]
    bottomQuat = [[0., 0., 0.71, -0.71], [0,0,1,0], [0,0,1,0], [0., 0., 0.71, 0.71]]



    j = json.load(open('allCuts.json', 'r'))
    
    if rON:

        
        tBase = [-48.36, 22.25, 243.39]
        
        tQuat = transformations.quaternion_about_axis(numpy.radians(22.5), [0,1,0])
        t = tBase +  2*numpy.dot(transformations.quaternion_matrix(tQuat), [0,0,1,1])[0:3]
        
        r = robot.Robot(zeroJoints='True', verbose=True)
        r.setTool([t, tQuat])
        r.setZone(finep=True)    
        
        
        
        wCart = [1104.4602685582297, 1199.630135398929, (472.1-50.8)]
        #wCart = [1127.6404205503334, 1146.7896910738002,  (472.1-50.8)]
        
        

        wQuat = transformations.quaternion_about_axis(0.90749940449, [0,0,1])
        r.setWorkObject([wCart, wQuat])
        

    tFlip = transformations.rotation_matrix(numpy.radians(180), [1,0,0])
    tRot = transformations.rotation_matrix(numpy.radians(180), [0,0,1])
    qRot = transformations.quaternion_about_axis(numpy.radians(180), [0,0,1])
    qFlip = transformations.quaternion_about_axis(numpy.radians(180), [1,0,0])

    for partSN, cutPlan in j.iteritems():
        if partSN <> '1007': continue
        else: print 'discarding part', partSN
        print 'starting on part number', partSN
        

        
        '''
        if rON: 
            tTarg = [350,0,50.8, 0,0,1,0]
            r.setJoints([45,1,1,2,86,2])
            r.setCartesian(tTarg)
            
            time.sleep(10)
            r.setJoints()
        '''

        numpy.set_printoptions(precision=2, suppress=True)        
        for cutSide in cutPlan:

            cutSide = numpy.array(cutSide)
            
            
            if numpy.average(cutSide[:,0]) > 0:
                cutSide[:,0:3] = transform(cutSide[:,0:3], tRot, onePad=True)[:,0:3]
                cutSide[:,3:7] = tQuatF(cutSide[:,3:7], qRot)

            cutSide = reduceRadius(cutSide)
            
            

            aTop = numpy.vstack(((cutSide[3:8]), cutSide[0]))
            aTop[:,3:7] = topQuat
            
            aBottom = numpy.array(cutSide[0:4])
            aBottom[:,3:7] = bottomQuat

            aBottom[:,0:3] = transform(aBottom[:,0:3], tFlip, True)
            aBottom[:,2] += 50.8
            #aBottom[:,3:7] = bottomQuat #rQuat(aBottom[:,3:7])
            
            
            leadIn = numpy.array(deepcopy(aTop[0]))
            leadIn[0] -= 5
            leadIn[2] -= 10

            leadOut = numpy.array(deepcopy(aTop[-1]))
            leadOut[0] -= 5
            leadOut[2] -= 5

            print '\nTop\n', aTop

            #print 'Executing Top Cuts'


            if rON:
                r.setJoints(ing)
                r.setSpeed(cutS)
                r.setCartesian(leadIn)
                if pON: r.plasmaOn(True)
                time.sleep(.5)
                for pt in aTop:
                    r.setCartesian(pt)
                r.setZone(finep=True)
                r.setCartesian(leadOut)
                if pON: r.plasmaOn(False)
                r.setSpeed()
                r.setJoints(eg)
                r.setJoints()
            raw_input('flip to the other side of this cut (about part axis)')


            leadIn = numpy.array(deepcopy(aBottom[0]))
            leadIn[0] -= 2

            leadOut = numpy.array(deepcopy(aBottom[-1]))
            leadOut[0] -= 2

            

            abM = aBottom[1:3]
            abM[:,0] += 1

            #abM[:,3:7] = tQuatF(abM[:,3:7], qRot)
            
            
            leadIn = numpy.array(deepcopy(abM[0]))
            leadIn[0] -= 2
            leadIn[1] +=10

            leadOut = numpy.array(deepcopy(abM[-1]))
            leadOut[0] -= 2
            leadOut[1] -=10

            print '\nBottom:\n', abM

            if rON:
                r.setJoints(ing)
                r.setSpeed(cutS)
                r.setCartesian(leadIn)
                if pON: r.plasmaOn(True)
                time.sleep(.5)
                for pt in abM:
                    r.setCartesian(pt)
                r.setZone(finep=True)
                r.setCartesian(leadOut)
                if pON: r.plasmaOn(False)
                r.setSpeed()
                r.setJoints(eg)
                r.setJoints()
            raw_input('flip to the other cut (perp to part axis)')


    
if __name__== '__main__':
    
    ing = [62.7,30.6,45.2,79.5,-42.2,-102.9]
    eg = [18,42,58,89,89,-58.4]
    veg = [18,10,10,89,89,-58.4]
    cutS = (35,100)

    tD = .75

    rON = '-r' in sys.argv
    pON = '-p' in sys.argv
 
    topQuat = [[0.,0., -0.71, 0.71], [0.,0., -0.71, 0.71], [0,0,1,0], [0,0,1,0], [0.,0., 0.71, 0.71], [0.,0., 0.71, 0.71]]
    bottomQuat = [[0., 0., 0.71, -0.71], [0,0,1,0], [0,0,1,0], [0., 0., 0.71, 0.71]]
    

    top, bottom = lamePath(267.3481, numpy.radians(67.5))
    print 'top:', top
    print 'bottom', bottom




    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    #ax.scatter(vecproj[:,0], vecproj[:,1], vecproj[:,2])
    



    if rON:
        
        tBase = [-48.36, 22.25, 243.39]        
        tQuat = transformations.quaternion_about_axis(numpy.radians(22.5), [0,1,0])
        t = tBase +  2*numpy.dot(transformations.quaternion_matrix(tQuat), [0,0,1,1])[0:3]

        


        r = robot.Robot(zeroJoints='True', verbose=True)
        r.setTool([t, tQuat])
        r.setZone(finep=True)    
        
        wCart = [1104.4602685582297, 1199.630135398929, (472.1-50.8)]
        wQuat = transformations.quaternion_about_axis(0.90749940449, [0,0,1])
        r.setWorkObject([wCart, wQuat])

        r.setJoints(ing)
        
    
    for cut in top:
        ax.plot(cut[:,0], cut[:,1], cut[:,2])

        if rON: 
            r.setCartesian(cut[0])
            r.setSpeed(cutS)
            time.sleep(tD)
            if pON: r.plasmaOn(True)
            time.sleep(tD)
        for pt in cut:
            if rON: r.setCartesian(pt)
        if pON: r.plasmaOn(False)
    if rON:
        r.setSpeed()
        r.setJoints(eg)
        r.setJoints()
        
    ax.plot(bottom[:,0], bottom[:,1], bottom[:,2])
    #plt.show()
    raw_input('flip to other side')


    if rON:
        r.setJoints(ing)
        
        
        r.setCartesian(bottom[0])
        time.sleep(tD)
        r.setSpeed(cutS)
        if pON: r.plasmaOn(True)
        time.sleep(tD)
        for pt in bottom:
            r.setCartesian(pt)
        if pON: r.plasmaOn(False)
        r.setSpeed()
        r.setJoints(veg)
        r.setJoints()
    
