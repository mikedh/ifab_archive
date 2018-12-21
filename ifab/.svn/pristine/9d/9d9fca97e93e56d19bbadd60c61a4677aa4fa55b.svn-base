import robot, transformations, jackDistance
import numpy, time, sys, math

import Gnuplot

#Reads position sensor, switches tool to welder, and then moves into position
ron = '-r' in sys.argv
v = '-v' in sys.argv
plot = '-p' in sys.argv





if ron: R = robot.Robot(verbose=v)
if plot: g = Gnuplot.Gnuplot()

inspectPoints = numpy.loadtxt('gageblock.txt')

weldQuat =  [0.43837115, 0. ,  0.89879405,  0.]
tq = [0,0,1,0]
q = transformations.quaternion_multiply(transformations.quaternion_about_axis(math.radians(35), [0,1,0]), [0,0,1,0])


if ron: 
    R.setToolFile('tool.object.displacement')
    L = robot.Logger(verbose=v)

try:
    R.setCartesian([inspectPoints[0], q])
    time.sleep(2)
    
    print "start record"
    L.start()
    for i in inspectPoints:
        if ron: R.setCartesian([i,q])
    cloud = L.stop(sequence=True)
    print "end record"
    


    if cloud <> None: 
        print 'recorded data count', len(cloud)
        if plot: 
            xyz0=numpy.mean(cloud, axis=0)
            M=cloud-xyz0
            g.splot(cloud)
            
        numpy.savetxt(('gage0' + '.txt'), cloud)

finally: 
    if ron: del R
    L.close()



q = raw_input('quit?')
