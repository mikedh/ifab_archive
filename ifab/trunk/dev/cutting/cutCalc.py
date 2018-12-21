import struct, numpy, json, sys, time, Queue, os
import timeit
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import quickhull, transformations
from copy import deepcopy
#import robot


class stlModel:
    #inital load part of class based on code from StackOverflow user 'DaClown'
    def __init__(self, key=None, filename=None):      
        self.normals = []; self.points = []; 
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
        self.normals = []; self.points = []
        self.triangles = []; self.bytecount = []
        self.fb = []

        try:
            f = open (infilename, "rb")
            self.read_header(f)
            l = self.read_length(f)
            try:
                while True:
                    self.read_triangle(f)
            except Exception, e: pass
            if len(self.triangles) <> l:
                print len(self.normals), len(self.points), len(self.triangles), l
        except Exception, e: pass

    def transform(self, tmat):
        if (numpy.shape(tmat) <> (4,4)): return False
        for i in xrange(len(self.points)):
            self.points[i] = (numpy.dot(tmat, numpy.append(self.points[i],1)))[0:3]
    

    def faceCross(self, face):
        a = self.points[face[1]]-numpy.array(self.points[face[0]])
        b = self.points[face[2]]-numpy.array(self.points[face[0]])
        cr = numpy.cross(a,b)
        area = numpy.linalg.norm(cr) / 2.0
        n = (cr / numpy.linalg.norm(cr))
        return (area, n)

    #http://mathworld.wolfram.com/Point-PlaneDistance.html
    def faceInFace(self, faceBase, faceTest, TOL=.01):
        a, n = self.faceCross(faceBase)
        
        xi = numpy.tile([self.points[faceBase[0]], self.points[faceBase[1]], self.points[faceBase[2]]], (3,1))
        x0 = numpy.vstack((numpy.tile(self.points[faceTest[0]], (3,1)), numpy.tile(self.points[faceTest[1]], (3,1)), numpy.tile(self.points[faceTest[2]], (3,1))))
        d = numpy.abs(numpy.dot((x0-xi), n))
        if numpy.sum(d) < TOL: 
            return True
        else: return False
                       
    def pointFaceDist(self, point, face):
        a, n = self.faceCross(face)
        xi = [self.points[face[0]], self.points[face[1]], self.points[face[2]]]
        x0 = numpy.tile(point, (3,1))
        d = numpy.abs(numpy.dot((x0-xi), n))
        return d[0]

    def pointInFace(self, point, face, TOL=.01):
        if self.pointFaceDist(point, face) < TOL: return True
        else: return False

    #find face areas with or without merging coplanar faces (slow), return an array that looks like:
    #[area, point index, normal vector, face list]
    def faceArea(self, fs=None, merge=False, verbose=False, TOL=.1):
        if fs==None: fs=self.triangles
        unused = numpy.ones(len(fs)); vecs = []; areas = []; fall = []
        for i, tri in enumerate(fs):
            if unused[i]: 
                faces = numpy.array(tri); unused[i] = 0
                sumArea, normal = self.faceCross(tri)
                if merge: 
                    for j, tri2 in enumerate(fs):
                        if unused[j]: 
                            if self.faceInFace(tri, tri2): 
                                unused[j] = 0
                                faces = numpy.vstack((faces, tri2))
                                a, n = self.faceCross(tri2)
                                sumArea += a
                vecs.append([tri[0], normal])
                areas.append(sumArea)
                fall.append(faces)
            if merge & verbose: print len(fs) - i
        return (areas, vecs, fall)

    def scalarProjection(self, vec, points=None):
        if points == None: points = self.points
        elif numpy.shape(points)[1] == 1: 
            pt = numpy.zeros((len(points), 3))
            for iz, ip in enumerate(points): pt[iz] = self.points[ip]
            points = pt
        elif numpy.shape(points)[1] <> 3: return None

        vec = vec / numpy.linalg.norm(vec)

        d = numpy.zeros(len(points))        
        for i,p in enumerate(points):
            d[i] = numpy.dot(p, vec)
        return d

    #returns list of point indexs in the plane of the face 

    #[   plane->[    face->[    vertex index->[
    #returns a list of len(planes) which holds vertex indices
    def segmentPoints(self, faces, faceIndex=None):
        unused = numpy.ones(len(self.points)); seg = []
        if faceIndex == None: faceIndex = xrange(faces)
        for i in faceIndex:
            tseg = []
            for j, p in enumerate(self.points):
                if unused[j]:
                    if self.pointInFace(p, faces[i][0]):
                        tseg.append(j)
                        unused[j] = 0
            seg.append(tseg)
        return seg
            
    #rotate partaxis to be along the X-axis, and normal along Z axis so it should be 2D, and then use quickhull
    def partEdges(self, vecBase, vecDir, partAxis, faces, faceIndex, kerfRadius=1):
        x = [1,0,0]; m = [0,0,1]; 
        
        #print faceIndex
        segPt = self.segmentPoints(faces, faceIndex)
        
        result = numpy.zeros((2,8,6))
        
        for faceCulled, faceLong in enumerate(faceIndex):
            newPts = numpy.zeros((len(segPt[faceCulled]),3))
            T = alignVectors(partAxis, vecDir[faceLong])
            
            Tinv = numpy.linalg.inv(T)

            for i, pInd in enumerate(segPt[faceCulled]):
                newPts[i] = numpy.dot(T, self.points[pInd])
                
            h = cleanHull(newPts[:,0:2])
            z = newPts[0,2]
 
            if (len(h) == 4) & (numpy.std(newPts[:,2]) < 1e-6): 
                
                iNX = numpy.argsort(h[:,0])
                for i in xrange(2):
                    result[i][faceCulled*2] = numpy.append(numpy.dot(Tinv, numpy.append(h[iNX[2*i]], z)), vecDir[faceLong])
                    result[i][faceCulled*2+1] = numpy.append(numpy.dot(Tinv, numpy.append(h[iNX[2*i+1]], z)), vecDir[faceLong])
            else: print 'error in hull set detected, hull is of length', len(h), 'Z stdev is', numpy.std(newPts[:,2])
            

                #plt.plot(h[:,0], h[:,1], 'o')
        #plt.axes().set_aspect('equal','datalim')
        #plt.show()
        #for i in xrange(2): result[i] = radialSort(result[i])
        
        for i in xrange(2): 
            result[i] = radialSort(result[i])
            
            result[i][:,0:3] = transform(result[i][:,0:3], T)
            result[i][:,3:6] = transform(result[i][:,3:6], T)

        
        newVB = transform(numpy.array(self.points)[[vecBase.flatten()]], T)
        ys = self.scalarProjection([0,1,0], newVB)
        xs = numpy.array([result[0][:,0], result[1][:,0], ]).flatten()
        zs = numpy.array([result[0][:,2], result[1][:,2], ]).flatten()
        exactCut = (numpy.max(xs)-numpy.min(xs)) + 2*kerfRadius


        yOFF = min(ys) + ((max(ys)-min(ys))/2)
        xOFF = min(xs) + ((max(xs)-min(xs))/2)
        zOFF = min(zs)
        result[0][:,0:3] -= [xOFF, yOFF, zOFF]
        result[1][:,0:3] -= [xOFF, yOFF, zOFF]

        
        
        

        xPos = numpy.int(numpy.average(result[0][:,0]) < numpy.average(result[1][:,0]))        
        result[xPos][:,0] += kerfRadius
        result[1-xPos][:,0] -= kerfRadius
        
        Teighty= numpy.eye(3); Teighty[0,0] = -1.0; Teighty[1,1] = -1.0
        graphable = deepcopy(result)
        result[xPos][:,0:3] = transform(result[xPos][:,0:3], Teighty)
        

        return result, exactCut 

    #takes the cartesian cut points/normals, and calculates robot orientations.
    def orientationCalc(self, cutPts, partAxis):
        numpy.set_printoptions(precision=3, suppress=True)
        res = numpy.zeros((2,8,7))
        cvs = numpy.zeros((8,3))
        for i, cut in enumerate(cutPts):
           
            cutC = numpy.average(cut[:,0:3], axis=0)
            cutN = numpy.cross((cut[0,0:3]-cutC), (cut[1,0:3]-cutC))
            cutN = cutN / numpy.linalg.norm(cutN)
            for j, pt in enumerate(cut):
                faceN = vecDirection(pt[3:6], pt[0:3]-cutC)
                faceN = faceN/numpy.linalg.norm(faceN)
                edgeVec = numpy.cross(cutN, faceN)
                cutVec = numpy.cross(cutN, edgeVec)
                cutVec = vecDirection(cutVec, -1*faceN)

                #cutVec = vecDirection(pt[3:6], 1*(pt[0:3]-cutC))
                #cutVec = cutVec / numpy.linalg.norm(cutVec)
                cvs[j] = faceN #cutVec
                
                quat = vec2quat2(cutVec, partAxis)
                
                res[i][j] = numpy.append(pt[0:3], quat)
                
                

            res[i] = reorder(res[i])
            
        print res

        
        
        '''
        plt.plot(cutPts[0][:,1], cutPts[0][:,2], '-')
        plt.plot(vecproj[:,1], vecproj[:,2], 'o')
        plt.show()
        plt.clf()
        plt.plot(vecproj[:,0], vecproj[:,2], 'o')
        plt.plot(cutPts[0][:,0], cutPts[0][:,2], '-')
        plt.show()
        '''

        vecproj = cutPts[1][:,0:3] + (cvs)*-10
        plotPts = numpy.vstack((cutPts[1], cutPts[1][0]))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(plotPts[:,0], plotPts[:,1], plotPts[:,2])
        ax.scatter(vecproj[:,0], vecproj[:,1], vecproj[:,2])
        plt.show()
       
        return res

    #we're going to be looking at the largest area planes, and looking for 2 sets of parallel planes, perpendicular to eachother. 
    #we also want the separation between the parallel sets to be as large as possible
    def cutCalc(self, lookAt=100):

        #first look at all faces, calculate area but don't merge
        a,v,f = self.faceArea(merge=False)
        sa = numpy.argsort(a)[-lookAt:]; st = True
        for i in sa: 
            if st: sface = f[i]; st=False
            else: sface = numpy.vstack((sface, f[i]))
        
        #now take the top lookAt largest by area faces, and calculate merged area
        na,v,f = self.faceArea(fs=sface, merge=True)
        sa = numpy.argsort(na); 
        

        #now take the top 8 largest faces after merging 
        basePt = numpy.zeros(8)
        #print 'smallest major face is', (na[sa[-8]]/na[sa[-9]]), 'x larger than next largest'
        st = True
        for i in sa[-8:]:
            
            if st: 
                vecBase= v[sa[i]][0]
                vecDir = v[sa[i]][1]
                area = na[sa[i]]
                faces = [(f[sa[i]]).tolist()]
                
                st = False
            else:
                vecBase= numpy.vstack((vecBase, v[sa[i]][0]))
                vecDir = numpy.vstack((vecDir, v[sa[i]][1]))
                area = numpy.vstack((area, na[sa[i]]))
                faces.append((f[sa[i]]).tolist())

        #find the unique abs normal vectors, there should be 2
        z = uniqueRows(numpy.round(numpy.abs(vecDir),5), return_index=True)

        #find the outwardmost planes (IE ditch the 4 inner faces)
        planeSet = numpy.zeros((2,2))
        for i in xrange(2):
            proj = self.scalarProjection(vecDir[z[i]], vecBase)
            planeSet[i] = [proj.argmin(), proj.argmax()]
        
            
        #find the unit vector pointing in the direction of the part axis
        partAxis = numpy.cross(vecDir[planeSet[0][0]],vecDir[planeSet[1][0]])
        if (numpy.sum(numpy.abs((partAxis - numpy.cross(vecDir[planeSet[0][1]],vecDir[planeSet[1][1]])))) > .001): 
            print 'WARNING: PART AXIS ERROR IS SUBSTANTIAL'
            raise NameError('AXISERROR')

        #print partAxis

        cuts, exactCut =  self.partEdges(vecBase, vecDir, partAxis, faces, numpy.int_(planeSet.flatten()))
        roughCut = cutLength(exactCut)

        self.cuts = self.orientationCalc(cuts, partAxis)
        print 'returned cuts are\n', self.cuts

        print 'exact cut length', numpy.round(exactCut,2), 'mm,', 'rough cut length', roughCut/25.4, 'inches'

        
        
        #try: robotMove(cuts)
        #except: pass
        return cuts.tolist()
    
        #numpy.savetxt('outDat.txt', numpy.vstack((graphable[0][:,0:3], graphable[1][:,0:3])))
        


def robotMove(cuts):
        r = robot.Robot(IP='localhost', zeroJoints=True)
        r.setSpeed((50,25))
        r.setTool([[41.3,22.4,256.97], transformations.quaternion_about_axis(numpy.radians(22.5), [0,1,0])])
        r.setZone(finep=True)
        
        print 'moving', cuts
        r.setWorkObject([[1500,0,1100], [1,0,0,0]])
        for i in cuts:
            
            for pt in cuts[i][4:8]:
                print pt
                r.setCartesian([pt[0:3],  pt[3:7]])
                
            #r.setJoints([ 7.18,   17.31,   43.1 ,   12.63, -110.16, -181.7])
            #for pt in bot:
                #r.setCartesian([pt[0:3],  pt[3:7]])
                    
            r.setJoints()
            raw_input('flipping')
        
            #r.setCartesian([cuts[i][0][0:3], cuts[i][0][3:7]])
          


def vectorAngle(v1=[ 0.99838777, 0.05676145, 0.], v2=[0,0,1]):
    v1 = v1/numpy.linalg.norm(v1); v2 = v2/numpy.linalg.norm(v2)
    cr = numpy.cross(v1,v2)
    th = numpy.arcsin(numpy.linalg.norm(cr)/(numpy.linalg.norm(v1)*numpy.linalg.norm(v2)))
    return (th,cr)


#accepts an (n,m) list of vectors and an (m, m) transformation matrix, and returns an (n,m) list of transformed vectors
def transform(v, T):
    return numpy.dot(T, numpy.array(v).transpose()).transpose()
    
#accepts (n,6) points and orders them based on angle. 
def radialSort(points, checkPlanar=True):
    sh = numpy.shape(points)
    c = numpy.average(points[:,0:3], axis=0)
    v = uniTize(points[:,0:3] - (numpy.tile(c, (sh[0],1))))
    
    n = numpy.cross(v[0], v[1]); n = n/numpy.linalg.norm(n)
    T = alignVectors(v[0], n, [1,0,0], [0,0,1])
    #planarPoints = numpy.zeros((sh[0],3))
    #for i in xrange(sh[0]): 
    planarPoints = transform(v,T)


    angs = numpy.arctan2(planarPoints[:,0], planarPoints[:,1])
    
    if checkPlanar:
        if numpy.std(planarPoints[:,2]) > .01: 
            print 'warning, non-planar cut detected!'
            #raise NameError('NON-PLANAR CUT')

    sInd = numpy.argsort(angs)
    res = points[[sInd]]
    
    '''
    planarPoints = planarPoints[[sInd]]
    plt.plot(planarPoints[:,0], planarPoints[:,1], '-')
    plt.show()
    '''
    return res
    

def cleanHull (points2D):
    hull2D = quickhull.qhull(points2D)
    n = numpy.size(hull2D,0);
    clHull = hull2D[0,:];
    #print n
    for i in xrange (0,n-2):
        p1 = hull2D[i,:];
        p2 = hull2D[i+1, :];
        p3 = hull2D[i+2, :];
        
        v2_unit = unitVector(p1,p2);
        v3_unit = unitVector(p2,p3)
        
        attribute = numpy.arccos(numpy.dot(v3_unit,v2_unit))*(180/numpy.pi);
        if (attribute > 5):
            clHull = numpy.vstack((clHull,p2));
        
    return clHull

def uniTize(a):
    r = numpy.zeros(numpy.shape(a))
    for i, e in enumerate(a):
        r[i] = e / numpy.linalg.norm(e)
    return r

def unitVector(p1,p2):
    v_p1 = p2 - p1;
    v_unit = v_p1 / numpy.linalg.norm(v_p1)
    return(v_unit)    



def parallel(v1,v2):
    PTOL = 1e-5
    if (abs(numpy.linalg.norm(v1)-numpy.linalg.norm(v2)) < PTOL): return True
    else: return False


def vec2quat(vec, rotation=numpy.radians(0)): 
    z0 = [0,0,1]; zq0 = [1,0,0,0]; a = [[0],[0],[1],[1]];
    if (len(vec) == 3):
        vec = vec / numpy.linalg.norm(vec)
        if (~parallel(vec,z0)):
            rotvec = numpy.cross(vec, z0); 
            rotangle = numpy.arcsin(numpy.linalg.norm(rotvec)/(numpy.linalg.norm(z0)*numpy.linalg.norm(vec)))
            rotquat = transformations.quaternion_about_axis(-rotangle, rotvec)
            quats = transformations.quaternion_multiply(rotquat, zq0)
        else: quats = zq0

        #check the rotation to see if we need to fix the sign by rotatating 180 degrees
        if numpy.sum(vec + numpy.dot(transformations.quaternion_matrix(quats), [0,0,1,1])[0:3]) < 1e-6:
            quats = transformations.quaternion_multiply(quats, [0,0,1,0])

        quats = transformations.quaternion_multiply(quats, transformations.quaternion_about_axis(rotation, [0,0,1]))

        return quats
    else: return False

def vec2quat2(vector, partAxis):
    
    partAxis = [-1,0,0]
    m = numpy.cross(vector, numpy.cross(partAxis, vector))
    
    m = m/numpy.linalg.norm(m)
    m = vecDirection(m, [-1,0,0])
    
    print 'avec', vector, m
    T = alignVectors(x1=vector, m1=m, x=[0,0,1], m = [1,0,0]) 
    q = transformations.quaternion_from_matrix(T)
    qr = transformations.quaternion_about_axis(numpy.radians(180), [0,0,1])
    qf = transformations.quaternion_multiply(q,qr)
    return q

#takes vec, and sees if its pointing in generalDirection. if it is, or if the direction/vec are perpendicular, it returns vec.
#if it is pointing in the opposite direction, it returns -vec
def vecDirection(vec, generalDirection):
    uvec = vec / numpy.linalg.norm(vec)
    generalDirection = generalDirection / numpy.linalg.norm(generalDirection)
    d = numpy.dot(uvec, generalDirection)
    if d == 0: return vec
    else: return numpy.sign(d)*numpy.array(vec)

# returns a (3,3) o transformation matrix from x>x1, m>m1, where x/x1 and m/m1 are perpendicular vectors 
# equation copped from http://steve.hollasch.net/cgindex/math/rotvecs.html
def alignVectors(x1, m1, x=[1,0,0], m = [0,0,1], homogeneous=False):
    c = numpy.cross(x,m); c1 = numpy.cross(x1,m1)
    A = numpy.array([x,m,c])
    B = numpy.array([x1,m1,c1])
    Tr = numpy.dot(numpy.linalg.inv(A), B)
    
    if homogeneous:
        T = numpy.eye(4)
        T[0:3,0:3] = Tr
        return T
    else:
        return Tr

#takes an (n,m), finds zero values, and groups them
#not general or particularly well written
def reorder(a):
    s = numpy.shape(a)
    z = numpy.int_(numpy.array(a)[:,2]) == 0
    if numpy.sum(z) == 2:
        ind = numpy.argmax(z)
        #a = numpy.roll(a, (numpy.shape(a)[0]-ind), axis=0)
        

        for i in xrange(10):
            if ((int(a[1][2]) == 0) & (int(a[2][2]) == 0)):
                return a
                
            else:
                a = numpy.roll(a, 1, axis=0)
        
        #return a

#find unique rows, using a random vector hash
def uniqueRows(a, return_index=True):
    dim = numpy.shape(a)[1]
    h = numpy.random.rand(dim); h = h/numpy.linalg.norm(h)
    u = numpy.unique(numpy.dot(a,h), return_index=True)[1]
    if return_index: return u
    else: return a[[u]]

    '''
        r = numpy.zeros((len(u), dim))
        for i in xrange(len(u)):
            r[i] = a[u[i]]
        return r '''

def cutLength(exactCut, minScrap=(25.4*1), increment=(25.4*2)):
    r = exactCut + minScrap
    r += increment - (r % increment)
    return r

def hashFN(flist):
    h = dict()
    sa = []; eCount = numpy.zeros(len(flist))
    for f in flist:
        try:
            key = f.split()[-2]
            if key=='-': key=f.split()[-3]
        except: key = f.split()[-1]
        if key in sa:
            ki = sa.index(key)
        else:
            ki = len(sa)
            sa.append(key)
        h[f] = str(ki*10).zfill(2) + str(int(eCount[ki])).zfill(2)
        
        eCount[ki] += 1
    
    return h

def main():
    hl = json.load(open('production/fileNames.txt', 'r'))
    os.chdir('stls')
    flist = os.listdir('.')
    h = hashFN(flist)
    
    if h.keys() == hl.keys(): print 'keys match'
    

    r = dict()
    for file in flist:
        if str(file).find('.STL') <> -1:
            m = stlModel(filename=file)
            print 'found', file
            
            try: r[h[file]] = m.cutCalc()
            except: print 'errored out'

    os.chdir('..')
    json.dump(r, open('allCuts.json', 'w'), sort_keys=True, indent=4)
    #json.dump(h, open('fileNames.txt', 'w'), sort_keys=True, indent=4)

    return True

def testmain():
    m = stlModel(filename='testMember.STL')
    r = m.cutCalc()

if __name__ == '__main__':
    t = time.time()
    testmain()

    
