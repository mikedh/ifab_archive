


import transformations 
import robot, projector 
import numpy, time, math, cPickle, sys, os

robON = '-r' in sys.argv
rLocal = '-l' in sys.argv
projON = '-p' in sys.argv

if rLocal: 
    RHOST = 'localhost'
    robON = True
    print 'local simulation mode \n'
else: RHOST = '192.168.125.1'


trunk = os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]


location0 = [1200,0,1200]
quats0 = [0,0,1,0]

location = location0 
quats = [0,0,1,0]


tool0 = [[0,0,0], [1,0,0,0]]
ctool = tool0


anglestep = math.radians(1)
linearstep = 1

pnum = 0
stepping = False



if robON:
    r = robot.Robot(IP=RHOST, toolfile=False, zeroJoints=True)
    r.setCartesian([location, quats])
    ctool = r.getTool()

if projON: 
    p = projector.Projector()
    p.send(projector.ccbGLTM())



#main loop
#continue statements in change commands let us not step when we change a parameter
vecrot = False
while 1:

    msg = raw_input("input:\t")
    
    if msg == "quit": break

    elif msg == "nostep": stepping = False

    elif msg == "step": 
        stepping = True
        continue

    elif msg == "stepa":
        try: anglestep = math.radians(float(raw_input("set parameter step value (degrees):\t")))
        except: pass
        continue

    elif msg == "stepl":
        try: linearstep = float(raw_input("set linear step value in mm:\t"))
        except: pass
        continue

    elif msg == "p":
        try: pnum = int(raw_input("Rotate around X/Y/Z axis: 0,1,2\nLinear Move X/Y/Z: 3,4,5\nVector Rotate (set 'vrot' first): 6\nTool X/Y/Z: 7/8/9\nparemeter = "))
        except: pass
        continue 

    elif msg == "go":
        a = raw_input("enter position as X,Y,Z: ")
        try: a = numpy.float_(a.split(','))
        except: continue
        if len(a) == 3: location = a
        if robON:
            r.setCartesian([location, quats])
        continue


    elif msg == "gov":
        a = raw_input("enter deltaZ: ")
        try: a = numpy.float(a)
        except: continue
        b = location[2] + a 
        if ((b > -250) & (b < 2000)): location[2] = b
        if robON: r.setCartesian([location, quats])
        continue

    elif msg == "setquats":
        a = raw_input("enter quats: ")
        try:  a = numpy.float_(a.split(','))
        except: continue
        if len(a) == 4: quats = a
        if robON: 
            r.setCartesian([location, quats])
        continue

    elif msg == "pquats":
        print quats

    elif msg == "zcirc":
        height = 750;
        cent = numpy.array([1250,0])
        table = robot.Target(numpy.append(cent, 0), [1,0,0,0])

        reachangle = numpy.radians(25)
        rad = height * numpy.sin(reachangle); 

        increment = 6
        theta = range(increment)*numpy.array([numpy.pi/(increment-1)])

        if projON: p.send(virtual.crosshairGLTM())


        vals = []
        for i in theta:
            headpos = cent + numpy.array([numpy.sin(i), numpy.cos(i)])*rad
            headpos = numpy.append(headpos, height)
            print 'going to position: ', headpos
            v = virtual.virtual(table, headpos)
            if robON: r.goTarget(v)
            try: offset = numpy.float_(raw_input("offset (x, y):\t").split(','))
            except: continue 

            dz = numpy.sqrt(numpy.sum(offset**2))/numpy.tan(reachangle)

            fline = numpy.append([reachangle, i], offset)
            fline = numpy.append(fline, dz)
            print  fline
            if (len(vals) == 0): vals = fline
            else: vals = numpy.vstack((vals, fline))
        numpy.savetxt('offsetvals.txt', vals)

    elif msg == "weldtool":
        if (robON): 

            ctool = cPickle.load(open(trunk + 'data/tool.object.welder', 'rb'))
            
            r.setTool(ctool)
            location = [1200,0,1000]
            quats = [0,0,1,0]
            r.setCartesian([location, quats])

    elif msg == "projtool":
        if (robON): 

            ctool = cPickle.load(open(trunk + 'data/tool.object.projector', 'rb'))
            
            r.setTool(ctool)
            location = [1200,0,1000]
            quats = [0,0,1,0]
            r.setCartesian([location, quats])

    elif msg == "basetool":
        if (robON): 
            ctool = tool0
            r.setTool(ctool)
            quats = quats0
            location = location0
            r.setCartesian([location, quats])
    elif msg == "dtool":
        tool = r.getTool()
        print 'dumping tool object to file ', tool
        f = open('tool.object', 'wb')
        cPickle.dump(tool, f)
        f.close()


    elif msg == "tool":
        a = r.getTool()
        print 'current tool: ', a
        b = transformations.quaternion_multiply([0,0,1,0], quats)
        toolquat = transformations.quaternion_multiply(a[1],b) 
        toolquat = transformations.quaternion_inverse(toolquat)
    
        tool = [a[0], toolquat]

        ctool = tool
        quats = quats0
        print 'setting tool: ', tool
        print 'moving to: ', [location,quats]
        if (robON):
            r.setTool(tool)
            #location = numpy.array(location) - toolcart
            r.setCartesian([location,quats])
        continue

    elif msg == "toolcart":
        
        stepin = raw_input("Enter the tool adjustment in base coordinates as X,Y,Z: ")
        try: tstep = numpy.float_(stepin.split(','))
        except: continue
        if len(tstep) == 3: tstep = numpy.append(tstep, 1)
        else: continue

        a = r.getTool()
        T = transformations.quaternion_matrix(a[1])
        Tinv = numpy.linalg.inv(T)
        P = transformations.quaternion_matrix(quats)
        BCP = numpy.dot(P, Tinv)
        print 'BCP: ', BCP
        
        
        tsprime = numpy.dot(tstep,BCP)
        a[0] += numpy.array(tsprime[0:3])
        print 'tstepPrime: ', tsprime
        
        print 'tool being set as: ', a
        r.setTool(a)
        r.setCartesian([location,quats])

    elif msg == "solid":
        if projON: glcomm.sendPickles(virtual.solidGLTM(), PHOST)


    elif msg == "xyh":

        print "d(x,y) values are based on starting the projector low, if high-low reverse signs"
        try:
            dx = float(raw_input("dx (mm):\t"))
            dy = float(raw_input("dy (mm):\t"))
            dh = float(raw_input("dheight (mm):\t"))
        except: continue
        c = math.sqrt((dx*dx) + (dy*dy))
        thr = math.atan(c/dh)
        print 'c: ', c, 'thr: ', thr

        #if ((dx > 0) ^ (dy > 0)): thr *= -1

        qrotation = transformations.quaternion_about_axis(-thr, [dy, -dx ,0])

        print 'current quats: ', quats, ' qrotation: ', qrotation
        if (~(dx==0) & ~(dy==0)):
            quats = transformations.quaternion_multiply(qrotation, quats)
        else: print "no rotation, zero values"
        if (robON): r.setCartesian([location, quats])

    elif msg == "vrot":
        vecrot = True
        try:
            dxg = float(raw_input('dx: '))
            dyg = float(raw_input('dy: '))
        except: continue

    elif msg == "pjoints":
        if robON: 
            r.setJoints([0,0,20,180,0,-180])
            location = [1016, 0, 1152]
            quats = [.57358,0,.81915,0]

    elif msg == "zcalib":
        try:
            d0 = float(raw_input("d0 (mm):\t"))
            h0 = float(raw_input("h0 (mm):\t"))
            d1 = float(raw_input("d1 (mm):\t"))
            h1 = float(raw_input("h1 (mm):\t"))
        except: continue

        tantheta = ((d1-d0)/2)/(h1-h0)
        spreadangle = 2 *numpy.arctan(tantheta)
        z01 = h1 - (d1/(2*tantheta))
        z00 = h0 - (d0/(2*tantheta))
        zc = (z01+z00)/2
        print "z calibration: \t" + str(zc) + "\t from z00:\t" + str(z00) + "\t z01:\t" + str(z01)   
        print 'spread angle: ', spreadangle


    elif len(msg) == 0:
        if (stepping):
            if pnum == 0:
                qrotation = transformations.quaternion_about_axis(anglestep, [1,0,0])
                quats = transformations.quaternion_multiply(qrotation, quats)
            elif pnum == 1:
                qrotation = transformations.quaternion_about_axis(anglestep, [0,1,0])
                quats = transformations.quaternion_multiply(qrotation, quats)
            elif pnum == 2:
                qrotation = transformations.quaternion_about_axis(anglestep, [0,0,1])
                quats = transformations.quaternion_multiply(qrotation,quats)
    
            elif (pnum in [3,4,5]):
                location[pnum-3] = location[pnum-3] + linearstep
            elif (pnum == 6) & (vecrot):
                qrotation = transformations.quaternion_about_axis(anglestep, [dyg,-dxg,0])
                quats = transformations.quaternion_multiply(qrotation, quats)

            elif (pnum in [7,8,9]) & robON:
                a = r.getTool()
                T = transformations.quaternion_matrix(a[1])
                Tinv = numpy.linalg.inv(T)
                P = transformations.quaternion_matrix(quats)
                BCP = numpy.dot(P, Tinv)
                print 'BCP: ', BCP
                
                tstep = [0, 0, 0, 1]
                tstep[pnum-7] += linearstep
                tsprime = numpy.dot(tstep,BCP)
                a[0] += numpy.array(tsprime[0:3])
                print 'tstepPrime: ', tsprime
                
                print 'tool being set as: ', a
                r.setTool(a)

            if (robON) & (location[2] > -215):
                print [location, quats]
                r.setCartesian([location, quats])


if robON: del r
