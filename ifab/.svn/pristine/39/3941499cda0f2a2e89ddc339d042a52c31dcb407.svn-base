import robot, numpy, transformations, math

R = robot.Robot(zeroJoints=True)
R.setToolFile('../data/tool.object.displacement')



c = numpy.array([1450,-400,-100])
r0 = 200; rmin = 50; count = 10; p = .75
q = transformations.quaternion_multiply(transformations.quaternion_about_axis(math.radians(-45), [1,0,0]), [0,0,1,0])

R.setJoints([-30.63, 61.03, 1.04, 103.96, 74.62, -122.0])
R.setZone('z10')
R.setSpeed((400,200))
cList = [[c+[-r0,0,0], q]]





for r in (((numpy.array(1.0)/range(1,5))**p)*(r0-rmin)+rmin):
    cList.append([c+[0,r,0], q])
    cList.append([c+[r,0,0], q])
    cList.append([c+[0,-r,0], q])
    cList.append([c+[-r,0,0], q])



cList = numpy.array(cList)

print cList
try:
    L = robot.Logger()
    
    L.start()
    R.circularList(cList)
    d = L.stop()

    numpy.savetxt('circle.dat', d)
finally:
    L.close()
    del R
