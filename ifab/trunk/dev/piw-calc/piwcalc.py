'''
Michael Dawson-Haggerty
CMU Robotics Institute
For Dr. David Bourne/iFAB

piwcalc.py: iFAB welding station calculator

usage: python piwcalc.py -files #JSON STRING FOR LIST OF FILE PATHS# -tmat #JSON STRING FOR TMAT 4x4 matrix#
'''




import struct, numpy, json, sys, time, Queue, os
import transformations


from quickhull import qhull
import ifabPCL as pcl
from matplotlib import pyplot as plt

from copy import deepcopy

class stlModel:
    #load part of class based on code from StackOverflow user 'DaClown'
    def __init__(self, key=None, keyroot='models/', filename=None):      
        self.normals = []; self.points = []; self.points0 = []
        self.triangles = [];self.bytecount = []
        self.fb = [] 
        if key <> None:
            p = os.path.join(keyroot, (str(key) + '.STL'))
            
            self.load(p)
            print 'loaded', key
            
            
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
        self.normals = []; self.points = []
        self.triangles = []; self.bytecount = []
        self.fb = []
        
        f = open (infilename, "rb")    
        self.read_header(f)
        l = self.read_length(f)
        try:
            while True:
                self.read_triangle(f)
        except:  pass
        if len(self.triangles) <> l:
            print len(self.normals), len(self.points), len(self.triangles), l
        self.points0 = deepcopy(self.points)
        

    def transform(self, tmat, tOriginal=False):
        if (numpy.shape(tmat) <> (4,4)): return False
        for i in xrange(len(self.points)):
            if tOriginal: self.points[i] = (numpy.dot(tmat, numpy.append(self.points0[i],1)))[0:3]
            else: self.points[i] = (numpy.dot(tmat, numpy.append(self.points[i],1)))[0:3]
    
    def project2D(self):
        return numpy.array(self.points)[:,0:2]
        
    def modelCorners (self):
        if (numpy.size(self.points) == 0): 
            print "Empty Array - No Corners Returned"
            return    
        P2D = self.project2D();
        noisyHull = qhull(P2D);
        convexHull = pcl.cleanHull(noisyHull);
        convexHull = numpy.vstack((convexHull,convexHull[0,:]));        
        return (convexHull);
    
    def modelPassEdge (self):
        Vertex = self.modelCorners();
        template = (Vertex, 0);
        template = numpy.delete(template,1);
        Edge = pcl.templateOutline(template);
        return (Edge)
        
    def projectPlane(self, flat, plane=[[ 1460.,-870.,-221.05733113], [ 1460.,700.,-223.52123963], [ 1820.,0., -224.42066831]]):
        mheight = numpy.mean(numpy.array(plane)[:,2])
        #skip the rotation, just translate
        tiled = numpy.tile(mheight, (numpy.shape(flat)[0],1))
        return numpy.hstack((flat, tiled))

    def virtual(self, worktarget=[945,0,0], cartesians=[945,0,1375]):
        # returns a virtual target that the robot can actually reach
        # virtual target will be pointed towards worktarget and located at cartesians
        z0 = [0,0,-1]; zq0 = [0,0,1,0]; a = [[0],[0],[1],[1]];
        if (len(cartesians) == 3) & (len(worktarget) == 3):
            dvec = unitv(worktarget-numpy.array(cartesians))
            if (~parallel(dvec,z0)):
                rotvec = numpy.cross(dvec, z0); 
                rotangle = numpy.arcsin(numpy.linalg.norm(rotvec)/(numpy.linalg.norm(z0)*numpy.linalg.norm(dvec)))
                rotquat = transformations.quaternion_about_axis(-rotangle, rotvec)
                virtualresult = [cartesians, transformations.quaternion_multiply(rotquat, zq0)]
            else: virtualresult = [cartesians, zq0]
            return virtualresult
        else: return False

    #pts is (n,2), function returns distance between consecutive points
    def edgeLength(self, pts):
        p = numpy.diff(pts, axis=0)
        lenEdges = numpy.sqrt(numpy.sum(p*p, axis = 1))
        return lenEdges

    def project(self, reachDist = 50, rHeight = 600):
        hull = self.modelCorners(); dmin = 1e9; ind = -1
        for i in numpy.argsort(self.edgeLength(hull))[-2:]:
            dist = numpy.abs(numpy.mean([hull[i], hull[i+1]], axis=0)[1])
            if dist < dmin: dmin = dist; ind = i
        projPt2D = hull[ind:ind+2,:]
        projPt3D = self.projectPlane(projPt2D)
        center = numpy.mean(projPt3D, axis=0)
        n = unitv(numpy.cross([0,0,1], numpy.diff(projPt3D, axis=0)))
        rPosition = center + 1*n*reachDist + [0,0,rHeight]
        vt = self.virtual(center, rPosition)   
        return vt

class piwCalc:
    def __init__(self, tMat, filekeys):
        self.items = dict()
        self.trunk =  os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]
        self.template = []


        p = os.path.join(self.trunk, 'workpieces', 'HMMWV', 'models')
        for f in filekeys:
            try: 
                m = stlModel(f, p)
                m.transform(tMat)
                self.add(f, m)
            except: print 'failed to load', f
            

    def cuts():
        pass

    def add(self, name, stl):
        if stl.__class__.__name__ == 'stlModel':
            self.items[str(name)] = stl
            
        else: print 'item not an stlModel'

    #finds an inspection plan which crosses all memebers
    def globalInspection(self):
        allHulls = dict()
        starting = True
        for name, model in self.items.iteritems():
            if starting:  pts = model.modelPassEdge(); starting = False
            else: pts = numpy.vstack((pts, model.modelPassEdge()))
        tq = [0,0,1,0]; plan = []
        xyz = pcl.compundedTaskInspection(pts, wf=1) 
        for i in xyz: plan.append([i, tq])
        return numpy.array(plan)
        
    def projection(self, key, reachDist = 50, rHeight = 600):
        return numpy.array(self.items[key].project(reachDist, rHeight))
    
    def genTemplate(self):
        starting = True
        for name, model in self.items.iteritems():
            if starting:  self.template = model.modelPassEdge(); starting = False
            else: self.template = numpy.vstack((self.template, model.modelPassEdge()))
       # pcl.plotTemplate(self.template)
        #plt.show()

    def getWelds(self):
        # extract welds directly from the template
        if self.template == []: self.genTemplate()
        idealWelds = pcl.extractWelds(self.template);
        n = numpy.size(idealWelds,0);
        zVal = numpy.zeros([n,1]) + (-170);
        idealWelds = numpy.hstack((idealWelds, zVal));


        return idealWelds
    
    # *Edge is percentage from the edge, so [0,0] would result in an unchanged weld
    #sideEdge is [top, bottom]. So a weld halfway down would be [0,.5]
    def calcWelds(self, topWelds=True, topEdge=[.1,.1], sideWelds=True, sideEdge=[0,.5], sHeight=50.8):

        topQuat = [0.43837115, 0, 0.89879405,0]
        sideQuat = topQuat
        w = self.getWelds()
        sT=True
        for i in xrange(0,numpy.shape(w)[0], 2):
            if topWelds:
                p0 = numpy.average(w[i:(i+2)], axis=0, weights=[1-topEdge[0], topEdge[0]])
                p1 = numpy.average(w[i:(i+2)], axis=0, weights=[topEdge[1], 1-topEdge[1]])
                if sT: welds = numpy.array([[p0, topQuat],[p1, topQuat]]); sT=False
                else: welds = numpy.vstack((welds, [[p0, topQuat],[p1, topQuat]]))
            if sideWelds:
                for j in xrange(2):
                    s0 = numpy.array([w[i+j], w[i+j]])
                    zs = [s0[0][2],(s0[0][2]-sHeight)]
                    s0[0][2] = numpy.average(zs, weights=[(1-sideEdge[0]), sideEdge[0]])
                    s0[1][2] = numpy.average(zs, weights=[sideEdge[1], (1-sideEdge[1])])
                    if sT: welds = [[s0[0], sideQuat], [s0[1], sideQuat]]; sT=False
                    else: welds = numpy.vstack((welds, [[s0[0], sideQuat], [s0[1], sideQuat]]))
        return welds
        

    def remQuat(self,l):
        r = numpy.zeros((len(l),3))
        for i in xrange(len(l)):
            r[i] = l[i][0]
        return r

    def spWelds(self):
        welds = self.remQuat(self.calcWelds(topWelds=True, topEdge=[0,0], sideWelds=False)); wl = len(welds)
        #self.removeClose(welds)
        plt.clf()
        sing = numpy.zeros(((wl/2),3))
        for i,w in enumerate(xrange(0,len(welds),2)):
            sing[i] = numpy.average(welds[w:(w+2)], axis=0)
            print 'dist', self.dist(welds[w:(w+2)])
            pass
        
        plt.plot(sing[:,0], sing[:,1], 'o')
        plt.show()
        return sing


    def removeClose(self, points):
        thresh = 1
        dup = numpy.zeros(len(points))
        for i in xrange(len(points)):
            print i
            
        e = edgeLength(points)
        print e

    def dist(self, pair):
        p=numpy.array(pair)
        #print 'p,', p
        return numpy.sum((p[1]-p[0])**2)**.5


    def distOrdering(self, w):
        starting = False
        for i in xrange(0,numpy.shape(w)[0], 2):
            if starting: ps = numpy.average(w[i:(i+2)], axis=0)
            else: ps = numpy.vstack((ps, numpy.average(w[i:(i+2)], axis=0)))
                                            

def edgeLength(pts):
    p = numpy.diff(pts, axis=0)
    lenEdges = numpy.sqrt(numpy.sum(p*p, axis = 1))
    return lenEdges


def unitv(v):
    v = numpy.array(v).flatten()[0:3]
    n = numpy.linalg.norm(v)
    if (n<>0): return v/n
    else: return v

def parallel(v1,v2):
    PTOL = 1e-2
    if (abs(numpy.linalg.norm(v1)-numpy.linalg.norm(v2)) < PTOL): return True
    else: return False



def assignID(path, start=1000):
    names = os.listdir(path)




if __name__ == '__main__':
    tMat = transformations.rotation_matrix(numpy.radians(-90), [1,0,0])
    

    top = [47,4,34,67,48,60,82,12,18,36,56,44,15,92,65,33,39]
    fn = ['hmmwv - Split1[' , ']-1']; keys = []
    for f in top: keys.append((fn[0] + str(f) + fn[1]))


    print keys

    c = piwCalc(tMat, keys)
    w = c.spWelds()

    j = {'sectionName': 'HMMWV top', 'tMat0':tMat.tolist(), 'weldPoints' : w.tolist()}

    json.dump(j, open('flatWelds.json', 'w'), sort_keys=True, indent=4)
    print j
    '''
    plan = c.globalInspection()
    for k in keys: print 'projections for', k,  c.projection(k)
    print 'inspection plan size', numpy.shape(plan)
    print 'welds', c.calcWelds()
    '''
