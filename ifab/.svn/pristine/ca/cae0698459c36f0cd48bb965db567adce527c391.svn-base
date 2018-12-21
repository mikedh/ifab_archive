import projector, robot
import os, numpy


p = projector.Projector()


r = robot.Robot(zeroJoints=True)
r.setJoints([-30.63, 61.03, 1.04,103.96, 74.62, -122.0])
trunk = os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]
r.setToolFile(trunk + 'data/tool.object.projector')

center = numpy.array([1500,0,200])
dq = [0,0,1,0]
r.setCartesian([center, dq])
p.send(projector.axisGLTM())

raw_input('got the axis? triangle is +x, square is +y')

box = 100
p.send(projector.ccbGLTM(box=box))

print 'checkerboard boxes are', box, 'pixels'

del r


