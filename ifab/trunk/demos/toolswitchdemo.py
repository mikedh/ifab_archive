import robot, transformations, jackDistance
import numpy, time, sys

#Reads position sensor, switches tool to welder, and then moves into position

ron = '-r' in sys.argv

if __name__ == '__main__':
    if ron: R = robot.Robot(zeroJoints=True, verbose=False)
    L = robot.Logger()

    if ron: 
        R.setToolFile('../data/tool.object.displacement')
        R.setCartesian([[ 1750. ,   0,  -100. ], [0, 0, 1, 0]])
        time.sleep(1)
  
    numpy.set_printoptions(precision=3)


    
    L.start()
    time.sleep(1)
    cloud = L.stop()
    L.close()

    if cloud <> None:
        if len(cloud) > 0:
            print 'recorded data count:', len(cloud)
            
            measuredPoint = numpy.array([numpy.average(cloud, axis=0), weldQuat])
            print 'initial value', measuredPoint

            #shitty way to force array byval (copy)
            measuredCloseApproach = list(1*measuredPoint)
            measuredFarApproach = list(1*measuredPoint)

            measuredCloseApproach[0][2] += 2
            measuredFarApproach[0][2] += 200

            print 'Far approach:', measuredFarApproach
            print 'Close approach:', measuredCloseApproach

            if ron:
                print 'set welder'
                R.setToolFile('../data/tool.object.welder')
                R.setCartesian(measuredFarApproach)
                R.setCartesian(measuredPoint)

                time.sleep(10)

                R.setCartesian(measuredFarApproach)
                R.setToolFile('../data/tool.object.displacement')
                R.setCartesian([[ 1750. ,   0,  -100. ], [0, 0, 1, 0]])
        else: print 'cloud length 0'


