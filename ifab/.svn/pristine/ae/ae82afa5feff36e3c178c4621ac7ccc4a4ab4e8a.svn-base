import struct, numpy, json, sys, time, Queue, os
import matplotlib.pyplot as plt

import transformations
import robot, projector, ifabPCL
from quickhull import qhull
import Gnuplot, pdb
import ifabPCL as pcl
from copy import deepcopy
import numpy as np


class stlModel:
    #load part of class based on code from StackOverflow user 'DaClown'
    def __init__(self, key=None, filename=None):      
        self.normals = []; self.points = []; self.filepoints = [] 
        self.triangles = [];self.bytecount = []
        self.fb = [] 

        if key <> None: self.load(('models/' + key + '.stl'))
        elif filename <> None: self.load(filename)

    def unpack (self, f, sig, l):
        s = f.read(l)
        self.fb.append(s)
        return struct.unpack(sig, s)

    def read_triangle(self, f):
        n = self.unpack(f,"<3f", 12)
        p1 = self.unpack(f,"<3f", 12)
        p2 = self.unpack(f,"<3f", 12)
        p3 = self.unpack(f,"<3f", 12)
        b = self.unpack(f,"<h", 2)
        self.normals.append(n)
        l = len(self.points)
        self.points.append(p1)
        self.points.append(p2)
        self.points.append(p3)
        self.triangles.append((l, l+1, l+2))
        self.bytecount.append(b[0])

    def read_length(self, f):
        length = struct.unpack("@i", f.read(4))
        return length[0]

    def read_header(self, f):
        f.seek(f.tell()+80)

    def write_as_ascii(self, outfilename):
        f = open(outfilename, "w")
        f.write ("solid "+outfilename+"\n")
        for n  in range(len(self.triangles)):
            f.write ("facet normal {} {} {}\n".format(self.normals[n][0],self.normals[n][1],self.normals[n][2]))
            f.write ("outer loop\n")
            f.write ("vertex {} {} {}\n".format(self.points[self.triangles[n][0]][0],self.points[self.triangles[n][0]][1],self.points[self.triangles[n][0]][2]))
            f.write ("vertex {} {} {}\n".format(self.points[self.triangles[n][1]][0],self.points[self.triangles[n][1]][1],self.points[self.triangles[n][1]][2]))
            f.write ("vertex {} {} {}\n".format(self.points[self.triangles[n][2]][0],self.points[self.triangles[n][2]][1],self.points[self.triangles[n][2]][2]))
            f.write ("endloop\n")
            f.write ("endfacet\n")
        f.write ("endsolid "+outfilename+"\n")
        f.close()

    def load(self, infilename):
        self.normals = []; self.points = []; 
        self.triangles = []; self.bytecount = []
        self.fb = []

        try:
            f = open (infilename, "rb")
            
            self.read_header(f)
            l = self.read_length(f)
            try:
                while True:
                    self.read_triangle(f)
            except:  pass
            if len(self.triangles) <> l:
                print len(self.normals), len(self.points), len(self.triangles), l
        except Exception, e: pass
        self.filepoints = deepcopy(self.points)

    def transform(self, tmat, transformOriginal = False):
        if (numpy.shape(tmat) <> (4,4)): return False

        if transformOriginal:
            for i in xrange(len(self.filepoints)):
                self.points[i] = (numpy.dot(tmat, numpy.append(self.filepoints[i],1)))[0:3]
        else:
            for i in xrange(len(self.points)):
                self.points[i] = (numpy.dot(tmat, numpy.append(self.points[i],1)))[0:3]



    def project2D(self):
        return numpy.array(self.points)[:,0:2]
        
    
    def modelCorners (self, plotting=False):
        
        if (numpy.size(self.points) == 0): 
            print "Empty Array - No Corners Returned"
            return 
            
        P2D = self.project2D();
        noisyHull = qhull(P2D);
        convexHull = pcl.cleanHull(noisyHull);
        convexHull = numpy.vstack((convexHull,convexHull[0,:]));
        
        
        if plotting: plt.plot(convexHull[:,0],convexHull[:,1],'-ro');
            
        return (convexHull);
    
    def modelPassEdge (self,flag=1):

        Vertex = self.modelCorners();
        template = (Vertex, 0);
        template = numpy.delete(template,1);
        Edge = pcl.templateOutline(template,keepAll = flag);

        return (Edge)
        
    def projectIngress(self):
        baseIngress =  [-8.48,62.97,0.35,167.4,51.42, -176.64]
        if len(numpy.shape(self.points)) <> 2: return baseIngress
        return calculateIngress(self.points, baseIngress)


    #pts is (n,2), function returns distance between consecutive points
    def edgeLength(self, pts):
        p = numpy.diff(pts, axis=0)
        lenEdges = numpy.sqrt(numpy.sum(p*p, axis = 1))
        return lenEdges


    def project(self, R, sDist = 50, sHeight = 600, plotting=False):
        p = projector.Projector()
        #points = self.project2D()
        hull = self.modelCorners(); dmin = 1e9; ind = -1
        for i in numpy.argsort(self.edgeLength(hull))[-2:]:
            dist = numpy.abs(numpy.mean([hull[i], hull[i+1]], axis=0)[1])
           
            if dist < dmin: dmin = dist; ind = i
        projPt2D = hull[ind:ind+2,:]
        projPt3D = projector.projectPlane(projPt2D)
        center = numpy.mean(projPt3D, axis=0)

        n = projector.unitv(numpy.cross([0,0,1], numpy.diff(projPt3D, axis=0)))
        rPosition = center + 1*n*sDist + [0,0,sHeight]
   
        vt = projector.virtual(center, rPosition)   
     
        seg = projector.Segments(projPt3D)

    
        gltm = projector.segProjections(vt,seg)
      
        trunk = os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]
        R.setToolFile(trunk + 'data/tool.object.projector')
        R.setCartesian(vt)

        p.send(gltm)
        p.doorOpen(1)

        if plotting:
            plt.plot(hull[:,0], hull[:,1], '.', hull[ind:ind+2,0],  hull[ind:ind+2,1], rPosition[0], rPosition[1], 'o')
            plt.show()
        else: raw_input('projecting member ')
        p.doorOpen(0)


    #dumb solution to the problem of fixtures not being within radius:
    #take n possible solutions, remove the invalid ones, and find the most widely spaced pair 
    def fixtureLine(self, pts, fixR=9.525, minR = 1100, maxR = 1800, slats = [[1036,1061]], scount=15):
        if numpy.shape(pts) <> (2,3): return None
        s = []
        
        
        
        n = projector.unitv(numpy.cross([0,0,1], numpy.diff(pts, axis=0)))
        for i in xrange(1,scount): 
            cand = (n*fixR + numpy.average(pts, axis=0, weights=[2,i]))
            cS = True
            cr = numpy.sqrt(numpy.sum(cand[0:2]**2))
            
            if (minR < cr < maxR):
                for j in slats: 
                    if (j[0] < cand[0] < j[1]): cS=False; break
            else: cS = False
            if cS: s.append(cand)

            cand = (n*fixR + numpy.average(pts, axis=0, weights=[i,2]))
            cS = True
            cr = numpy.sqrt(numpy.sum(cand[0:2]**2))
            if (minR < cr < maxR):
                for j in slats: 
                    if (j[0] < cand[0] < j[1]): cS=False; break
            else: cS = False
            if cS: s.append(cand)

        if numpy.shape(s)[0] == 2: return s
        if numpy.shape(s) == (0,): return None
        #print 'valid points', s
        r = distantPts(s)
        #print 'most distant', r
        return r

    def projectFixtures(self, R, radius=9.525, sHeight = 400, plotting=False):
        p = projector.Projector()
        

        if plotting: plt.clf()

        hull = self.modelCorners(); dmax=0; ind = -1
        for i in numpy.argsort(self.edgeLength(hull))[-2:]:
            dist = numpy.abs(numpy.mean([hull[i], hull[i+1]], axis=0)[1])
        if dist > dmax: dmax = dist; ind = i
        projPt2D = hull[ind:ind+2,:]
        projPt3D = projector.projectPlane(projPt2D)

        weighting = numpy.array([[3,1], [1,3]])

        n = projector.unitv(numpy.cross([0,0,1], numpy.diff(projPt3D, axis=0)))
        trunk = os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]
        R.setToolFile(trunk + 'data/tool.object.projector')
        
        minR=1100; maxR=1800


        #for w in weighting:
        fixL = self.fixtureLine(projPt3D)
        if fixL <> None:
            for pt in fixL:
                #pt = n*radius + numpy.average(projPt3D, axis=0, weights=w)
                if minR < numpy.sqrt(numpy.sum(pt[0:2]**2)) < maxR:

                    if plotting: plt.plot(pt[0], pt[1], 'ko')

                    rPosition = pt  + [0,0,sHeight]
                    vt = projector.virtual(pt, rPosition)   

                    gltm = projector.circleGLTM(r=20)

                    R.setCartesian(vt)
                    p.send(gltm)
                    p.doorOpen(1)
                    raw_input('projecting fixture ')
                else: print 'skipped fixture at', pt

        ptN = -1
        if minR < numpy.sqrt(numpy.sum(projPt3D[0][0:2]**2)) < maxR: ptN = 0
        elif minR < numpy.sqrt(numpy.sum(projPt3D[1][0:2]**2)) < maxR: ptN = 1
        
        if ptN <> -1: 
            pt = projPt3D[ptN]
            p.send(projector.crosshairGLTM())
            rPosition = pt  + [0,0,sHeight]
            vt = projector.virtual(pt, rPosition)   
            
            R.setCartesian(vt)
            raw_input('projecting endpt')

        if plotting:
            plt.plot(hull[:,0], hull[:,1], '.', hull[ind:ind+2,0],  hull[ind:ind+2,1])
            plt.show()

        p.doorOpen(0)

        


#takes a list of models and inspects them, returning 'some stuff'
class Inspector:
    def __init__(self):
        self.items = dict()
        self.clouds = dict()
        self.ingress = dict()
        self.trunk =  os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]
        self.ideal2actual = numpy.eye(4)

    def add(self, name, stl, ingress):
        if stl.__class__.__name__ == 'stlModel':
            self.items[str(name)] = stl
            self.ingress[str(name)] = ingress
        else: print 'item not an stlModel'

    def inspectGlobal(self, R=None, L=None, plotting=True):
        if R.__class__.__name__ == 'Robot':
            starting = True
            R.setSpeed((100,50))
            plan = self.inspectionPlanGlob()
            if (plan <> None) & (plan <> []): 
                
                if (R.setToolFile(self.trunk + 'data/tool.object.displacement') == False): return False
                R.setJoints(self.ingress[self.items.keys()[0]])
                if starting: R.setSpeed(); starting = False
                
                if L == None: L = robot.Logger()
                R.setZone('z10')
                R.setSpeed((400,170))
                L.start()
                R.circularList(plan)
                time.sleep(.5)
                self.clouds['glob'] = L.stop(sequence=True)
                try: print 'recorded', len(self.clouds['glob']), 'points for', name
                except: print 'recorded no data'
            R.setSpeed()
            R.setZone()
        if L <> None: L.close()
        print 'tm'
        
        
    def inspectZig(self, R, L=None):
        print 'entering zig inspect'
        if R.__class__.__name__ == 'Robot':
            starting = True
            R.setSpeed()
            for name, model in self.items.iteritems():
                plan = self.inspectionPlanZig(name, model)
                if (plan <> None) & (plan <> []): 
                    print 'now inspecting', name, 'plan shape', numpy.shape(plan), 'model shape', numpy.shape(model.points)                
                    if (R.setToolFile(self.trunk + 'data/tool.object.displacement') == False): return False
                    R.setJoints(self.ingress[name])
                    if starting: R.setSpeed(); starting = False
                    if R.setBuffer(plan, gotoFirst=True):
                        if L == None: L = robot.Logger()
                        R.setZone('z10')
                        R.setSpeed((200,50))
                        time.sleep(1)
                        L.start()
                        R.executeBuffer()
                        time.sleep(.5)
                        self.clouds[name] = L.stop(sequence=True)
                        R.setSpeed()
                        try: print 'recorded', len(self.clouds[name]), 'points for', name
                        except: print 'recorded no data'
                R.setZone()
        if L <> None: L.close()
        #self.templateMatch()
        
    def runSegments(self,R):
        self.generateTemplate()
        welds = ifabPCL.extractWelds(self.template)
        segs, checkIdx = ifabPCL.confirmWeldPath(self.template,transformationMat=self.ideal2actual, normalFactor= 29, lengthFactor=30 )
        
        plt.show()
        self.inspectSegments(segs,checkIdx, welds, R=R)
        
        
    #accepts a list of segments of dimensions (n, 2, 3)
    #returns a list of point clouds of dimensions (n, pointCount, 3)
    def inspectSegments(self, segments, checkIdx, welds, R=None, L=None, inspectHeight=-130, plotting=True):
        tableHeight = -223

 ################################
    
        self.clouds = dict()
        starting = True; 
        if R==None: R = robot.Robot() #zeroJoints=True)
        if (R.setToolFile(self.trunk + 'data/tool.object.displacement') == False): return False
        #
        R.setSpeed()
        R.setJoints([-30.63, 61.03, 1.04, 103.96, 74.62, -122.0])

        numpy.set_printoptions(precision=2, suppress=True)
        print 'inspecting segments', numpy.array(segments), '\n\n'

        qFF = transformations.quaternion_about_axis(-numpy.radians(45), [1,0,0])
        qBase = transformations.quaternion_multiply(qFF, [0,0,1,0]) 

##################################
       
        weldPos = welds[0];

        N1 = numpy.size(weldPos,0);
        N2 = np.arange(0,N1,2);
        N3 = N2 + 1;
        
        weld_y_A = weldPos[N2[checkIdx > 0],1];
        weld_y_B = weldPos[N3[checkIdx > 0],1];
        
        reachCorner  = np.hstack((N2[weld_y_A >= weld_y_B], N3[weld_y_B > weld_y_A]))
        reachCorner = reachCorner[np.argsort(reachCorner)];
        
        weldActual = weldPos[reachCorner,:];
        weldChecked = weldActual * 0;
        counterA = 0;
        
        if L==None: L = robot.Logger()
        for cloudNum, seg in enumerate(numpy.array(segments)):    

            if int(numpy.average(seg)) == 0: continue

            #angle i've tested from -60:90 roughly
            angRange = [numpy.radians(-80), numpy.radians(80)]

           # segVec = numpy.cross((seg[1]-seg[0]), [0,0,1])
           # if segVec[1] < 0: segVec *= -1
            segVec = seg[2]
            

            tempAngle = numpy.rad2deg(numpy.arctan2(segVec[1],segVec[0]))
            print "temp Angle ", tempAngle
            if (tempAngle > 0):
                tempAngle = -180 + tempAngle 
                

            lineAngle = numpy.deg2rad(tempAngle + 90)
            
            print "calculated Approach ", numpy.rad2deg(lineAngle)
        
           ####################### Test for angle ################
            
            print 'calculated segment angle',  numpy.degrees(lineAngle)
            #lineAngle = insideRange(lineAngle, angRange)

            
            rQ = transformations.quaternion_about_axis(lineAngle, [0,0,1])
            quats = transformations.quaternion_multiply(rQ, qBase)

            rVec = numpy.dot(transformations.quaternion_matrix(quats), [0,0,1,1])[0:3]
            rVec = rVec / numpy.linalg.norm(rVec); 
            starting=True
            
            pathBuffer = np.zeros([2,2]);
            
            for i, pt in enumerate(seg[0:2]):
                #                
                pathBuffer[i,:] = pt[0:2]
                #                
                x = (inspectHeight - (pt[2] + tableHeight))/rVec[2]
                rPos = (pt + [0,0,tableHeight]) + x*rVec
                numpy.set_printoptions(precision=2, suppress=True)
                print 'robot cartesian', rPos, rVec, quats 
                R.setCartesian([rPos, quats])
               
                if starting:
                    R.setSpeed((20,50))
                    time.sleep(0.7);
                    L.start()
                    time.sleep(0.7);
                    starting=False
            
            d = L.stop(sequence=True)
            time.sleep(0.7)
            R.setSpeed();
            bufferUnit = pcl.unitVector(pathBuffer[0,:], pathBuffer[1,:]);
            dCount = np.size(d,0);
            grpCount = dCount/10
                       
            weldChecked[counterA,:] = pcl.coupleData(d, jntAngle = 27, group = grpCount, step = 30)[2];
                 # determine point along path wherein the seam was located
                #pathMag = bufferUnit * weldChecked[counterA,0];
            
                # move to seam position projected along path 
                #posXY = pathBuffer[0,:] + pathMag           
            
                # use depth value to move along approach vector to hit expected seam
                #depthMove = segVec[0:2] * (weldChecked[counterA,1] + 200)
                #weldChecked[counterA,:] = posXY # + (depthMove)
            
           # except:
           #     print 'Error in seamExtraction encountered'
           #     weldChecked[counterA,:] = np.zeros([2]);
           
            counterA += 1;
            self.clouds[str('segments' + str(cloudNum))] =  d
            #for name, cloud in self.clouds.iteritems():
        
        print 'Actual Welds: \n\n', weldActual;
        print 'Checked Welds: \n\n ', weldChecked;

        computedOffset = weldChecked - weldActual;
        self.checkedWeldIdx = reachCorner;
        self.checkedOffset = computedOffset;
        
        
        
        plt.figure();
        pcl.plotTemplate(self.template);
        plt.plot(weldChecked[:,0], weldChecked[:,1], 'ko')
        if L <> None: L.close()
        

    def generateTemplate(self):
        starting = True
        for name, model in self.items.iteritems():  
            if starting: 
                c = model.modelPassEdge()
                if len(c) == 8: 
                    self.template = c
                    starting = False
            else: 
                c = model.modelPassEdge()
                if len(c) == 8:
                    self.template = numpy.vstack((self.template, c))
                

    def templateMatch(self):
        starting = True
        for name, cloud in self.clouds.iteritems():
            if starting: p3d = ifabPCL.extractEdges(cloud); starting = False
            else: p3d = numpy.vstack((p3d,ifabPCL.extractEdges(cloud)))
        
        self.generateTemplate()
        #templateList = list();
       
        p2d = p3d[:,[0,1]];
        zVal = p3d[:,2];
        zSurf = zVal[zVal > numpy.mean(zVal)];
        zSurfAvg = numpy.mean(zSurf)
        self.zHeight = zSurfAvg
        
        print " Printing Expected Height \t", self.zHeight, "\n"
        #template = ifabPCL.templateOutline(templateList, 1)
        
        
        self.ideal2actual = ifabPCL.multiCoreTemplateFit(p3d, deepcopy(self.template), cores = 7, flip = 0);
        #self.ideal2actual = numpy.eye(4)
        #elf.template = ifabPCL.flipTemplate(self.template);
        print "Transformation Matrix ", self.ideal2actual

        # now transform template?
        
        
                
    #this needs to return the list of welds with no other dicking around
    def getWelds(self):
        #self.templateMatch()
        # extract welds directly from the template 
        self.generateTemplate()
        idealWelds0 = ifabPCL.extractWelds(self.template)
        idealWelds = idealWelds0[0];
        
        n = numpy.size(idealWelds,0);
        zVal = numpy.zeros([n,1]) + (-168);
        idealWelds = numpy.hstack((idealWelds, zVal));
        outweld = numpy.array(idealWelds)

        '''
        
        n0 = np.arange(0,n);
        nA = self.checkedWeldIdx;
        n0[nA] = 9999;
        nB = n0[n0 <> 9999];

        # ######################## Hack'em Here ####################

        
        idealWelds[nA,0:2] = idealWelds[nA,0:2] + self.checkedOffset;
        idealWelds[nB,0:2] = idealWelds[nB,0:2] + self.checkedOffset;

        idealWelds = numpy.array(idealWelds)
        print idealWelds
        ############################################################
        #idealWelds = transform(idealWelds, self.ideal2actual, onePad=True)
        '''

        
        print 'idealwelds\n\n', idealWelds
        return (idealWelds, idealWelds0[1])


    # *Edge is percentage from the edge, so [0,0] would result in an unchanged weld
    #sideEdge is [top, bottom]. So a weld halfway down would be [0,.5]
    def calcWelds(self, topWelds=True, topEdge=[.1,.1], sideWelds=False, sideEdge=[0,.5], sHeight=50.8):

        topQuat = [0.43837115, 0, 0.89879405,0]
        sideQuat = topQuat
        w,ang = self.getWelds();
        
        #w = alignweld(w[:,0:2],self.ideal2actual)
        #print '\n\nWELDS\n',  w, '\n\n', ang
        minR = 1000; maxR=2100; r = []
        sT=True; gS = True
        for i in xrange(0,numpy.shape(w)[0], 2):
            
            if not (minR < numpy.sqrt(numpy.sum(w[i][0:2]**2)) < maxR): continue
            if topWelds:
                p0 = numpy.average(w[i:(i+2)], axis=0, weights=[1-topEdge[0], topEdge[0]])
                p1 = numpy.average(w[i:(i+2)], axis=0, weights=[topEdge[1], 1-topEdge[1]])
                if sT: welds = [numpy.append(p0, topQuat),numpy.append(p1, topQuat)]; sT=False
                else: welds = numpy.vstack((welds ,  [numpy.append(p0, topQuat),numpy.append(p1, topQuat)]))
            if sideWelds:
                for j in xrange(2):
                    s0 = numpy.array([w[i+j], w[i+j]])
                    zs = [s0[0][2],(s0[0][2]-sHeight)]
                    s0[0][2] = numpy.average(zs, weights=[(1-sideEdge[0]), sideEdge[0]])
                    s0[1][2] = numpy.average(zs, weights=[sideEdge[1], (1-sideEdge[1])])
                    
                    if sT: welds = [numpy.append(s0[0], sideQuat), numpy.append(s0[1], sideQuat)]; sT=False
                    else: welds = numpy.vstack((welds, [numpy.append(s0[0], sideQuat), numpy.append(s0[1], sideQuat)]))
            if gS: gWeld = [welds]; gS = False
            else: gWeld.append(welds)
            sT = True
        
        return numpy.array(gWeld)


    def alignWeld(self, weld, model2ideal):
        #loop through clouds, and return the location of the weld.
        outweld = numpy.array(weld)
        px = []; py = []
        for index, waypoint in enumerate(outweld): 
            outweld[index][0] = numpy.dot(model2ideal, numpy.append(outweld[index][0],1))[0:3]
            outweld[index][0] = numpy.dot(self.ideal2actual, numpy.append(outweld[index][0],1))[0:3]
            if (self.zHeight > -173):
                outweld[index][0][2] = self.zHeight
            else: outweld[index][0][2] = -168
            px.append(outweld[index][0][0])
            py.append(outweld[index][0][1])        
        return outweld

    def loadClouds(self, key):
        for name in numpy.append(self.items.keys(), 'glob'):
            try: 
                self.clouds[name] = numpy.loadtxt(self.trunk + 'data/scans/cloud-' + str(name) + key + '.txt')
                print 'loaded cloud', name, numpy.shape(self.clouds[name]), 'for model size', numpy.shape(self.items[name].points)
            except: print 'skipped cloud', name

    def dump(self):
        for name, cloud in self.clouds.iteritems():
            print 'saved', name
            if cloud <> None:
                numpy.savetxt((self.trunk + 'data/scans/cloud-' + str(name) + time.strftime('%d-%H%M') + '.txt'), cloud)
            else: print 'save err:', name, 'is empty.'
        self.clouds = dict()

    


    def clear(self):
        self.items = dict()
        self.clouds = dict()

    #finds an inspection plan which crosses all memebers
    def inspectionPlanGlob(self):
        allHulls = dict()
        starting = True
        c = 0
        for name, model in self.items.iteritems():
            
            if starting:  pts = model.modelPassEdge(); starting = False
            else: pts = numpy.vstack((pts, model.modelPassEdge()))
            print "printing name ",name

        tq = [0,0,1,0]; plan = []
        #xyz = ifabPCL.Calculate_ZigZag_Pattern(pts, widthFactor=.3)
        xyz = ifabPCL.compundedTaskInspection(pts,  wf=40) 
        for i in xyz: plan.append([i, tq])
        return numpy.array(plan)

    #assumes the model is a tubular member, so that a line fit 
    #finds the approximate axis of the member.
    #output is a list of xyz points to inspect (n,3) 
    def inspectionPlanZig(self, name, model):
        #this is a placeholder function which loads 
        pts = model.project2D()
        tq = [0,0,1,0]; plan = []
        xyz = ifabPCL.Calculate_ZigZag_Pattern(pts, passes=15, widthFactor=1.1, lengthFactor=.4)

        
        print 'skipping member', numpy.sqrt(numpy.sum(numpy.average(xyz, axis=0)[0:2]**2))
        if numpy.sqrt(numpy.sum(numpy.average(xyz, axis=0)[0:2]**2)) < 1200: 
            
            return None
        for i in xyz: plan.append([i, tq])
        return numpy.array(plan)

    def close(self):
        try: self.L.close()
        except: pass

    def __del__(self):
        self.close()


def jogCorrect(R, wp0):
    pass


def distantPts(pts):
    l = len(pts); dmax = 0; res = []
    for i in xrange(l):
        for j in xrange(l):
            d = numpy.sqrt(numpy.sum((pts[i]-pts[j])**2))
            if d > dmax: 
                dmax = d
                res = numpy.vstack((pts[i], pts[j]))
    return res


def weldIngress(weld):
    baseIngress =  [13.5, 40.5, 23.5, 0.8, -18.8, -1.1 ]
    return calculateIngress(weld, baseIngress)

def calculateIngress(points, baseIngress):
    cart = numpy.average(points[0:2], axis=0)
    ang = numpy.degrees(numpy.arctan2(cart[1], cart[0]))
    if (-45 < ang < 45):
        baseIngress[0] = ang
    return baseIngress

def vectorAngle(v1=[ 0.99838777, 0.05676145, 0.], v2=[0,0,1]):
    n1 = numpy.linalg.norm(v1) 
    n2 = numpy.linalg.norm(v2)
    if (n1<>0) & (n2<>0):
        v1 = v1/n1
        v2 = v2/n2
    else: return 0
    cr = numpy.cross(v1,v2)
    th = numpy.arcsin(numpy.linalg.norm(cr)/(numpy.linalg.norm(v1)*numpy.linalg.norm(v2)))
    return th


def transform(points, T, onePad=False):
    if len(numpy.shape(points)) == 1:
        return numpy.dot(numpy.append(points, numpy.zeros(int(onePad))),T)
    if onePad: 
        points = numpy.hstack((points, numpy.ones((numpy.shape(points)[0], 1))))
        return (numpy.dot(T, numpy.array(points).transpose()).transpose())[:,0:(numpy.shape(T)[1]-1)]
    return (numpy.dot(T, numpy.array(points).transpose()).transpose())



# vrange is min, max
def insideRange(value, vRange):
    if value < vRange[0]: return vRange[0]
    elif value > vRange[1]: return vRange[1]
    else: return value


def measurePlane(R, L, height):
    tq = [0,0,1,0]
    ipt = [[1460,-870,-100], [1460,700,-100], [1820,0,-100]]
    plane = []

    if (R.setToolFile('C:/Users/mikedh/Dropbox/cmu/svnmain/trunk/data/tool.object.displacement') == False): return False
    for i in ipt:
        print i
        R.setCartesian([i,tq])
        time.sleep(2)
        L.start()
        time.sleep(1)
        plane.append(numpy.mean(L.stop(), axis = 0))


    return numpy.array(plane)


if __name__ == '__main__':

    pt = numpy.random.random((10,3))
    print distantPts(pt)
    
    
