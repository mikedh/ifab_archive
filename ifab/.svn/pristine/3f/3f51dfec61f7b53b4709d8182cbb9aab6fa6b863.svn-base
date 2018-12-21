import robot, transformations, jackDistance
import numpy, time, sys

#Reads position sensor, switches tool to welder, and then moves into position

ron = '-r' in sys.argv
weld = '-w' in sys.argv
v = '-v' in sys.argv



#pts = [[1476.6, -804.4, -170], [1474.1, -851.5, -170] ]


 
 
pts = 1000 * numpy.array([[1.4874,  -0.8060,  -0.168], [1.4882, -0.8563, -0.168]])

print pts



weldApproach = [1400, -800, 200]
starting = True

weldQuat =  [0.43837115, 0. ,  0.89879405,  0.]
dq = [0,0,1,0]
goTarget = []

if __name__ == '__main__':
    if ron: 
        R = robot.Robot(verbose=v)
        goTarget = [[pts[0], weldQuat], [pts[1], weldQuat]]
        print goTarget

        R.setCartesian([weldApproach, dq])

        R.setToolFile('tool.object.welder')
        R.setBuffer(goTarget)
        R.executeBuffer()

        R.setCartesian([weldApproach, dq])
    

    
            
            

    
