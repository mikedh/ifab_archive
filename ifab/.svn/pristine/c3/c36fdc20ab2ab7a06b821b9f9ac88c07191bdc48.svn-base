import projector, robot
import os, numpy

r = robot.Robot(zeroJoints=True)
r.setJoints([-30.63, 61.03, 1.04,103.96, 74.62, -122.0])
trunk = os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]
r.setToolFile(trunk + 'data/tool.object.projector')
center = numpy.array([1200,-500,-223])
vt = projector.virtual(center, (center + [0,0,750]))
r.setCartesian(vt)
virtual = robot.Target(vt[0], vt[1])
hlen = 25
print 'vt:', vt
p = projector.Projector()
p0 = p.pixloc((center + [hlen,-hlen,0]), virtual)
p1 = p.pixloc((center + [hlen,hlen,0]), virtual)
p2 = p.pixloc((center + [-hlen,hlen,0]), virtual)
p3 = p.pixloc((center + [-hlen,-hlen,0]), virtual)
g = (projector.boxGLTM(numpy.array([p0,p1,p2,p3]).flatten()))
print g.verts, g.faces
p.send(g)
del r


