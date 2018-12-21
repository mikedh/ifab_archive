import robot, transformations, jackDistance
import numpy, time, sys

#Reads position sensor, switches tool to welder, and then moves into position

ron = '-r' in sys.argv
weld = '-w' in sys.argv
v = '-v' in sys.argv

pts = [[ 1500. ,   600,  -100. ], [ 1550. , 600  ,  -100. ]]
weldApproach = [1500,600,200]
starting = True

weldQuat =  [0.43837115, 0. ,  0.89879405,  0.]
dq = [0,0,1,0]
goTarget = []

if __name__ == '__main__':
    if ron: R = robot.Robot(verbose=v)
    L = robot.Logger(verbose=v)

    if ron: 
        R.setToolFile('tool.object.displacement')
        for i in pts:
            R.setCartesian([i, dq])
            time.sleep(1)
            L.start(); time.sleep(1)
            cloud = L.stop()
            if cloud <> None:
                if len(cloud) > 0:
                    measuredPoint = numpy.array([numpy.average(cloud, axis=0), weldQuat])
                    if starting: goTarget = measuredPoint; starting = False
                    else: goTarget = numpy.vstack((goTarget, measuredPoint))
        print goTarget
        L.close()
        R.setCartesian([weldApproach, dq])

        print "2 seconds until weld"
        time.sleep(2)
        print 'welding'
        R.setToolFile('tool.object.welder')
        R.setBuffer(goTarget)
        if weld: R.weldBuffer()
        else: R.executeBuffer()
        R.setCartesian([weldApproach, dq])
    

    
            
            

    
