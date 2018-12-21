from transformations import *
#from quickhull import qhull
import struct
import numpy as np
import collections, copy
try:  import scipy.spatial as spatial; sP=True
except: sP=False


class TriMesh():
    def __init__(self):
        self.name = ''
        self.points = []
        self.faces = []
        self.edges = []

    def transform(self, T):
        self.points = transform(self.points, T)

    def crossSection(self, origin, direction, TOL=1e-3):
        d0 = (np.dot(self.points[[self.edges[:,0]]]-origin, direction))
        d1 = (np.dot(self.points[[self.edges[:,1]]]-origin, direction))

        #edge passes cleanly through plane
        hits = np.nonzero(np.sign(d0) <> np.sign(d1))[0] 
        #edge lies on the crosssection plane
        onPlane = np.nonzero((np.abs(d0) < TOL) & (np.abs(d1) < TOL))[0]

        hits = np.append(hits, onPlane)
        p0 = self.points[[self.edges[[hits]][:,0]]]
        p1 = self.points[[self.edges[[hits]][:,1]]]
        
        pointpairs = planeLine(origin, direction, p0, p1)
        res = mergePointPairs(pointpairs, return_index=False)
        
        
        return res

    def rayIntersect(self, rays):
        return None

class TriMeshAssembly():
    def __init__(self, path=None):
        self.models = dict()
        if path <> None: self.addPath(path)

    #thinking about getting rid of this.
    def fileKeys(self, flist):
        delin = [' ', '-', ',', '.']; d = delin[0]
        for dq in delin[1:]: 
            if (len(flist[0].split(dq)) > len(flist[0].split(d))): 
                d = dq
        digits = len(flist[0].split(d))

    def applyAll(self, func):
        self.results = dict()
        for name, model in self.models.iteritems():
            self.results[name] = func(model)
        return self.results
        
    def addPath(self, path = '.'):
        for f in os.listdir(path):
            if str(f).upper().find('.STL') <> -1:
                self.models[f] = StlMesh(filename=os.path.join(path,f))


class StlMesh(TriMesh):
    def __init__(self, filename=None, filetype='binary'):
        if filename <> None: self.load(filename, filetype)
    
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
        self.faces.append((l, l+1, l+2))
        self.bytecount.append(b[0])

    def read_length(self, f):
        length = struct.unpack("@i", f.read(4))
        return length[0]

    def read_header(self, f):
        f.seek(f.tell()+80)

    def write_as_ascii(self, outfilename):
        f = open(outfilename, "w")
        f.write ("solid "+outfilename+"\n")
        for n  in range(len(self.faces)):
            f.write ("facet normal {} {} {}\n".format(
                    self.normals[n][0],self.normals[n][1],self.normals[n][2]))
            f.write ("outer loop\n")
            f.write ("vertex {} {} {}\n".format(self.points[
                        self.faces[n][0]][0],self.points[
                        self.faces[n][0]][1],self.points[
                        self.faces[n][0]][2]))
            f.write ("vertex {} {} {}\n".format(self.points[
                        self.faces[n][1]][0],self.points[
                        self.faces[n][1]][1],self.points[
                        self.faces[n][1]][2]))
            f.write ("vertex {} {} {}\n".format(self.points[
                        self.faces[n][2]][0],self.points[
                        self.faces[n][2]][1],self.points[
                        self.faces[n][2]][2]))
            f.write ("endloop\n")
            f.write ("endfacet\n")
        f.write ("endsolid "+outfilename+"\n")
        f.close()

    def generateEdges(self):
        st = True; self.points = np.array(self.points)
        e = []
        for f in self.faces:
            if st: 
                e = np.array([np.sort(f[0:2]), np.sort(f[1:3]), np.sort([f[0], f[2]])])
                st=False            
            else:
                e = np.vstack((e, [np.sort(f[0:2]), np.sort(f[1:3]), np.sort([f[0], f[2]])]))
        self.edges = e


    def load(self, filename, filetype='binary'):
        self.name = filename
        if filetype == 'binary': self.loadBinary(filename)
        elif filetype == 'ascii': self.loadAscii(filename)
        else: 
            if detectBinaryFile(filename): self.loadBinary(filename)
            else: self.loadAscii(filename)
        self.generateEdges()

    def loadAscii(self, filename):
        f = open(filename, 'r')
        name = f.readline()
        for line in f:
            pass
        f.close()

    def loadBinary(self, filename):
        self.normals = []; self.points = []
        self.faces = []; self.bytecount = []
        self.fb = []

        try:
            f = open (filename, "rb")
            self.read_header(f)
            l = self.read_length(f)
            try:
                while True:
                    self.read_triangle(f)
            except Exception, e: pass
            if len(self.faces) <> l:
                print len(self.normals), len(self.points), len(self.faces), l
        except: print 'loading of', filename, 'failed'


#http://stackoverflow.com/questions/898669/how-can-i-detect-if-a-file-is-binary-non-text-in-python
def detectBinaryFile(filename):
    textchars = ''.join(map(chr, [7,8,9,10,12,13,27] + range(0x20, 0x100)))
    fbytes = open(filename).read(1024)
    return bool(fbytes.translate(None, textchars))
    
def planeLineSingle(planeOri, planeDir, lineOri, lineDir):
    lineDir = unitVector(lineDir)
    planeDir = unitVector(planeDir)
    d = np.dot((planeOri-lineOri), planeDir) / np.dot(lineDir, planeDir)
    return lineOri + lineDir*d
    
#takes a plane origin/direction, and 2 (n,3) lists of points defining line segments
def planeLine(planeOri, planeDir, pt0, pt1):
    lDir = unitVector(pt1-pt0)
    planeDir = unitVector(planeDir)
    t = np.dot(planeDir, np.transpose(planeOri-pt0))
    b = np.dot(planeDir, np.transpose(lDir))
    d = np.array(t / b)
    return pt0 + np.reshape(d,(np.shape(lDir)[0],1))*lDir


def pointLine(lineOri, lineDir, points):
    if np.shape(points) == (3,):
        pt = points 
        return np.linalg.norm(np.cross((pt - lineOri), (pt - (lineOri+np.array(lineDir)))))/np.linalg.norm(lineDir)
    else:
        d = []
        for pt in points:
            d.append(np.linalg.norm(np.cross((pt - lineOri), (pt - (lineOri+np.array(lineDir)))))/np.linalg.norm(lineDir))
        return d

    
def transform(points, T, onePad=False):
    if len(np.shape(points)) == 1:
        return np.dot(np.append(points, np.zeros(int(onePad))),T)
    if onePad: 
        points = np.hstack((points, np.ones((np.shape(points)[0], 1))))
        return (np.dot(T, np.array(points).transpose()).transpose())[:,0:(np.shape(T)[1]-1)]
    return (np.dot(T, np.array(points).transpose()).transpose())
    

def unitVector(points):
    if len(np.shape(points)) == 1: 
        return points / np.linalg.norm(points)
    else: 
        return points / (np.reshape(np.sqrt(np.sum(points**2, axis=1)), (np.shape(points)[0],1)))
    

def majorAxis(points, returnError=False):
    sq = np.dot(np.transpose(points), points)
    d, v = np.linalg.eig(sq)
    if returnError:
        pass
    else:
        return v[np.argmax(d)]



#takes a curve on an arbitrary plane, and returns its centroid
#http://stackoverflow.com/questions/1023948/rotate-normal-vector-onto-axis-plane
def polygonArea3D(points, return_centroid=True, TOL=1e-4):
    origin = np.array(points[0])
    ptVec = points - origin

    loc = np.argmax(np.sum(ptVec ** 2, axis=1))
    normal = (np.cross((points[loc] - origin), (points[1] - origin)))
    
    axis0 = unitVector(np.cross((points[1]-origin), normal))
    axis1 = unitVector(np.cross(normal, axis0))
    
    pr0 = np.dot(ptVec, axis0)
    pr1 = np.dot(ptVec, axis1)

    #polygon area is only valid in 3D if all points are in the same plane
    planarCheck = np.dot(ptVec, normal)
    if np.std(planarCheck) > TOL: return False

    points2D = np.transpose([pr0, pr1])
    if return_centroid:
        A, C = polygonArea2D(points2D, return_centroid=True)
        C3D = (C[0] * axis0) + (C[1] * axis1) + origin
        return np.abs(A), C3D
    else: 
        return polygonArea2D(points2D, return_centroid=False)

#http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
def polygonArea2D(pts, return_centroid=True):
    A = 0.0; C = np.array([0.0,0.0])
    for i in xrange(len(pts)-1):
        Ai = ((pts[i][0]*pts[i+1][1]) - (pts[i+1][0]*pts[i][1]))
        A += Ai
        if return_centroid:
            Ci = [Ai * (pts[i][0] + pts[i+1][0]), Ai * (pts[i][1] + pts[i+1][1])]
            C += Ci
    A  *= .5
    if return_centroid:
        C *= (1 / (6*A))
        return A, C
    return A
    

# returns a (3,3) transformation matrix from x>x1, m>m1, where x/x1 and m/m1 are perpendicular vectors 
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


def uniqueRows(a, return_index=True, decimals=5, tries=2):
    dim = np.shape(a)[1]; up = []
    u  = np.unique(np.round(np.dot(a,np.random.rand(dim)), decimals), return_index=True)[1]
    if return_index: return u
    else: return a[[u]]


#similar to uniqueRows except with KD trees so we can do a distance tolerance
def uniquePoints(points, return_index=False, TOL=1e-4):
    if sP == False: return None
    t = spatial.KDTree(points)
    n = np.shape(points)[0]
    checked = np.zeros(n)
    pos = 0; uind = []
    while (not np.all(checked) & (pos < n)):
        
        if checked[pos]: pos += 1; continue
        d,i = t.query(points[pos], k = 4, distance_upper_bound=TOL)
        print 'pos:', pos, 'checked:', checked, 'uind:', uind, 'i:',  i
        checked[[(i<>n)]] = 1 
        uind.append(pos); pos += 1
    return unid

#returns extrusion axis of a triMesh model
#extrusion is defined as a constant cross section swept along a path
#thus, extrusuion cross sections which 
#we identify thin pieces first, by their being thinner than a threshold amount 
def extrusionAxis(mesh, tries=5):
    axisGuess = majorAxis(mesh.points)
    
    proj = np.dot(mesh.points, axisGuess)
    extents = [mesh.points[np.argmin(proj)], mesh.points[np.argmax(proj)]]
    st = True; 

    for i in xrange(tries):
        loc = np.average(extents, axis=0, weights=[(tries+1-i), i+1])
        
        pts = mesh.crossSection(loc, axisGuess)
        aMax = 0; cMax = []
        for xyz in pts:
            a, c = polygonArea3D(xyz, return_centroid=True)
            if a > aMax:
                cMax = c
                aMax = a
        if st:
            cents = cMax
            areas = aMax
            st = False
        else:
            cents = np.vstack((cents, cMax))
            areas = np.vstack((areas, aMax))


    c = np.average(cents, axis=0)
    m = majorAxis(cents)

    d = pointLine(c, m, cents)
    print d




# if curves are [A, B, C, B] it returns [A, B, C]
def mergePointPairs(pp, return_index=True):
    
    def indexPair(index):
        if index % 2 == 0: return index + 1
        else: return index - 1 
    
    up = [];  curves = []
    consumed = np.zeros(np.shape(pp)[0]); 
    h  = np.round(np.dot(pp,np.random.rand(np.shape(pp)[1])), 6)
    s = np.argsort(h)
    pos = 0; ccurve = []
    for ti in xrange(len(h)):
        if not consumed[pos]:
            ccurve = np.append(ccurve, pos)
            consumed[pos] = 1
        pair = indexPair(pos)
        if not consumed[pair]:
            ccurve = np.append(ccurve, pair)
            consumed[pair] = 1
            index = (h == h[pair])
            index[pair] = 0
            pos = np.argmax(index)
        elif consumed[pos]:
            if len(ccurve) > 0: curves.append(ccurve)
            else: break
            ccurve = []
            pos = np.argmin(consumed)
    if return_index:
        return np.int_(curves)
    else:
        curvePts = []
        for c in np.int_(curves):
            curvePts.append(pp[[c]])
        return curvePts
    
        

#planeSVD:
#http://www.lsr.ei.tum.de/fileadmin/publications/KlasingAlthoff-ComparisonOfSurfaceNormalEstimationMethodsForRangeSensingApplications_ICRA09.pdf
def surfaceNormal(points):
    return np.linalg.svd(points)[2][-1]

def radialSort(points, origin=None, normal=None):
    if origin==None: origin = np.average(points, axis=0)
    if normal==None: normal = surfaceNormal(points)
    
    axis0 = [normal[0], normal[2], -normal[1]]
    axis1 = np.cross(normal, axis0)
    ptVec = points - origin
    pr0 = np.dot(ptVec, axis0)
    pr1 = np.dot(ptVec, axis1)
    angles = np.arctan2(pr0, pr1)
    return points[[np.argsort(angles)]]
               
