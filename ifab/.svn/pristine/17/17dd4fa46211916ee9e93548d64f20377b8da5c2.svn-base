import numpy, cPickle, zlib, socket, sys, os.path, cPickle, time, math
import transformations , ue9, robot

class Projector:
    def __init__(self, IP='127.0.0.1' , PORT=50007, resolution=[800,600], spreadangle=[.53, 0.8799], rotation=numpy.radians(180), shear=0, openDoor=False):

        
        self.IP = IP
        self.PORT = PORT
        self.res = resolution
        self.ang = spreadangle
        self.shear = shear
        self.rot = rotation
        if openDoor: self.doorOpen(1)


    def send(self, object): 
        sendPickles(object, self.IP, self.PORT)

    def rcent(self): 
        cent = numpy.array(self.res)/2
        
        return cent


    def doorOpen(self, doorState=0):
        try: d = ue9.UE9()
        except: return False

        doorval = [60600, 58000] #[closed, open]
        doorindex = int(int(doorState)>0)
        d.timerCounter(UpdateConfig = True, NumTimersEnabled = 1,  TimerClockBase = 4, TimerClockDivisor = 15,  Timer0Mode = 0, Timer0Value = doorval[doorindex])
        time.sleep(.5)
        d.timerCounter(UpdateConfig = True, NumTimersEnabled = 0)
        d.close(); del d


    def pluralpixloc(self, points, virtual):
        plen = numpy.shape(points)[0]
        res = numpy.zeros((plen, 2))
        for i in range(plen):
            res[i] = self.pixloc(points[i], virtual)
        return res

    # returns a pixel location, based on projector location, point in space, and projector info
    def pixloc(self, point, virtual):
        ray = self.projectray(point, virtual)
        pix = self.vec2pix(ray)
        pix = numpy.int_(pix)
        return pix

    # returns a 'ray' from the projector base (tv) to a point 
    # rays can then be tranlated into a pixel value from the angular information on the projector
    def projectray(self, point, virtual):
       
        tv  = numpy.append((point-numpy.array(virtual.cart)),1); 
        #rotation matrix to bring point into projectors frame
        r = numpy.linalg.inv(transformations.quaternion_matrix(virtual.quat))
        rp = transformations.rotation_matrix(self.rot, [0,0,1])
        r = numpy.dot(rp,r)
        #do rotation
        ray = r*numpy.matrix(numpy.reshape(tv, (4,1))); 
        #print 'ray: ', ray
        ray[2] *=-1
        return ray[0:3]


        # returns the endpoint pixel location from a vector (or 'ray', defined in this function as 'v'), based on conversion to angular coordinates, 
        # and a linear angular - pixel mapping. This may be wrong. 
        # PROJ defines the native resolution, in pixels of the projector, followed by the angular spread in radians

    def vec2pix(self, v):
        #return self.matrixInterp(v)
        return self.squareInterp(v)
    def squareInterp(self, v, ang=[.6812,.50747], res=[800,600]):
        v = numpy.array(v).flatten()[0:3]
        cent = numpy.array(res)/2; 
        pxrad = numpy.array(res)/(ang)
        r = numpy.sqrt(numpy.dot(v,v))
        if r == 0: return 0
        if v[2] == 0: return 0
        v = v/r
        angs = numpy.array([numpy.arcsin(v[1]/v[2]), numpy.arcsin(v[0]/v[2])])
        res = (angs*pxrad) + cent
        return res.flatten()[0:2]

    def matrixInterp(self, vec, tMat= [-5.00183893e+00, -8.23612244e+02, 3.32666103e+01, 3.22275775e+01,-8.30083831e+02, 3.60730484e-01, 2.04487553e+01, 1.97933201e+01, 6.53988022e-02,-2.55144876e-03, 5.59581867e-02,-2.87582278e-01], res=[800,600]):
        
        #tMat = numpy.array(tMat)
        m = numpy.reshape(tMat, ((3,4)))
        vec = vec/numpy.linalg.norm(vec)
        vec = numpy.append(numpy.array(vec).flatten()[0:3],[1])
        
        pix = numpy.dot(m, vec)
        pix = pix / pix[2]
        pix = pix[0:2]+(numpy.array(res)/2)
        return pix


class GLtriangleMessage:
    def __init__(self, verts = [], faces = [], color =[0,255,0], period = .1, label=('', 200,200)):
        self.verts = verts
        self.faces = faces

        self.colors = []
        self.setColors(color)

        self.label = label
        self.period = period
        

    def makeInt(self):
        self.faces = numpy.int_(self.faces)
        self.verts = numpy.int_(self.verts)
    def setColors(self, color=[0,0,255]):
        
        if (len(color) == 3):
            vs = numpy.shape(self.verts)
            if len(vs) == 1: vc = (vs[0])/2
            else: vc = (vs[1])/2
            self.colors = list(color)*vc
        else: self.colors = color

#Segments class defines a continuous line segment, defined by verticies
#Each segment also has a direction vector, which represents the direction the arrow will point
#verts are shaped (n,3), vecs are (n-1, 3) shaped

class Segments: 
    def __init__(self, segverts=[], segvectors=[], thickness=20, arrowthick = 50, normal=[0,0,1]):

        self.verts = segverts

        #the direction the arrows point is the opposite direction that we 'extrude' the arrows/lines
        if ((segvectors <> [])): self.vecs = numpy.array(segvectors)*-1
        else: self.perpVec(normal)

        self.thick = thickness
        self.athick = arrowthick

    def perpVec(self, normal):
        count = numpy.shape(self.verts)[0]-1
        perp = numpy.ones((count,3))
        for i in xrange(count):
            sv = self.verts[i+1] - numpy.array(self.verts[i])
            perp[i] = numpy.cross(sv, normal)
        self.vecs = perp

def validSegments(segs):
    if (segs.__class__.__name__ == 'Segments'):
        vertshape = numpy.shape(segs.verts); vecshape = numpy.shape(segs.vecs)
        if ((len(vertshape) == 2) & (len(vecshape) == 2)):
            if ((vertshape[1] == 3) & (vecshape[1] == 3) & (vertshape[0] == (vecshape[0]+1))):
                return True
            else:
                print "err: shapes wrong"
                print vertshape
                print vecshape
                return False
        else:
            print "err: dimensions incorrect"
            return False
    else:
        print 'err: not a segment' 
        return False
        
def GLtmAppend(a,b):
    if (GLtmCheck(a) & GLtmCheck(b)):
        ash = numpy.shape(a.verts); bsh = numpy.shape(b.verts)
      
        if (len(ash) == 1): 
            vcount = ash[0]/2
            if (len(bsh) == 1): verts = numpy.append(a.verts, b.verts)
            if (len(bsh) == 2): verts = numpy.hstack((numpy.tile(a.verts, (bsh[0],1)), b.verts))

        elif (len(ash) == 2): 
            vcount = ash[1]/2
            if (len(bsh) == 1): verts = numpy.hstack((a.verts, numpy.tile(b.verts, (bsh[0],1))))
            elif ((len(bsh) == 2) & (bsh[0] == ash[0])): verts = numpy.hstack((a.verts, b.verts))
            
        faces = numpy.append(a.faces, (numpy.array(b.faces)+vcount))
        colors =  numpy.append(a.colors, b.colors)
        c = GLtriangleMessage(verts,faces, colors)
        return c
    else:
        if GLtmCheck(a):
            #print 'append failed, returning first value passed'
            return a
        elif GLtmCheck(b):
            #print 'append failed, returning last value passed'
            return b
        else:
            print 'append failed, returning nothing'
            return None

def GLtmCheck(gltm):
    #sanity check gltm
    #check: class name, len(verts)%2==0, face indicies don't exceed bounds, facecount%3==0 
 
    if (gltm.__class__.__name__ == 'GLtriangleMessage'):
        

        #Check vert dimensions consistant 
        tshape = numpy.shape(gltm.verts)
        if (len(tshape) == 1): vcount = tshape[0]
        elif (len(tshape) ==2 ): vcount = tshape[1]
        else:
            print 'GLTM check failed on incorrect vert shape'
            return False

        if (vcount == 0): 
            #print 'zero vertex count'
            return False

        #check that verts are 2D (pairs)
        if ((vcount%2) <> 0): 
            print 'GLTM check failed, verts are not pairs'
            return False

        #check face dimensions
        fshape = numpy.shape(gltm.faces)



        #not supporting varied face configs, we're just updating verts
        #check that faces is 1D, then make sure it divides into triangles
        if (len(fshape) <> 1):
            print 'GLTM check failed on non- 1D faces'
            return False
        if ((fshape[0]%3) <> 0): 
            print 'GLTM check failed on non triple faces'
            return False
        
        #check that face indexes within bounds
        if not numpy.all([(j < (vcount/2)) for j in numpy.array(gltm.faces).flatten()]):
            print 'GLTM check failed on out of bound faces'
            return False

        if (((vcount*3)/2) <> numpy.shape(gltm.colors)[0]): gltm.setColors()


        return True

    else:
        print 'class name incorrect:', gltm.__class__.__name__
        return False


# send pickled message to GL server
# serializes, compresses, appends header, and sends object
# object should be GLtriangleMessage
def sendPickles(obj, HOST, PORT):
    HEADERLENGTH = 16
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    pobj = zlib.compress(cPickle.dumps(obj))
    pobjlen = str(len(pobj)).zfill(HEADERLENGTH)

    s.sendall((pobjlen + pobj))
    s.close()


def boxGLTM(v0): #, proj=Projector(), centered=True):
    
    f0 = numpy.array([0,1,2,0,2,3])
    
    res = GLtriangleMessage(v0,f0)
    return res

def pointGLTM(point, size=1):
    point = numpy.array(point)
    verts = [(point+[0,size]), (point+[-size,-size]), (point+[size,-size]) ]
    verts = numpy.array(verts).flatten()
    faces = [0, 1, 2]
    res = GLtriangleMessage(verts,faces)
    return res

def ccbGLTM(box=100, proj=Projector(), crosshairs=True, send=False):
    
    a = crosshairGLTM(2,(box/2))
    a.setColors([255,0,0])

    b = checkerboardGLTM(box, proj, True)
    b.setColors([0,255,0])

    if crosshairs: gltm = GLtmAppend(b,a )
    else: gltm = b


    if send: 
        proj.send(gltm)
        return numpy.int_(numpy.array(proj.resolution)/box)
    else: return gltm



def checkerboardGLTM(box=4, proj=Projector(), centered=True):
    v0 = numpy.array([[0,0], [box,0], [box,box], [0,box]])
    f0 = numpy.array([0,1,2,0,2,3])
    count = 0; ic = 0; centoffset = [0,0]

    if centered: centoffset =  (numpy.array(proj.rcent()) % box)

    verts = []; faces = []
    for j in xrange(-box, proj.res[1]+box, box):
        ic += 1
        for i in xrange(-box, proj.res[0]+box, 2*box):
            verts = numpy.append(verts, (v0+centoffset+[(i+(ic%2)*box),j]).flatten())
            faces = numpy.append(faces, f0+(count*4))
            count += 1
    res = GLtriangleMessage(verts,faces)
    return res

def solidGLTM(PROJ=[[848, 480], [0.73, 0.33]]):
    verts = [0,0,     PROJ[0][0],0,      PROJ[0][0],PROJ[0][1],      0,PROJ[0][1]]
    print verts 
    faces = [0,1,2, 0,3,2]
    res = glcomm.GLtriangleMessage(verts, faces)
    return res


def crosshairGLTM(pixo=2, pixlen=100, proj=Projector()):
    cent = proj.rcent()
    vertsv = numpy.array([[cent+[pixo,pixlen]], [cent+[-pixo, pixlen]], [cent+[pixo, -pixlen]], [cent+[-pixo, -pixlen]]])
    vertsh = numpy.array([    [cent+[pixlen, pixo]] ,  [cent+[pixlen, -pixo]], [cent+[-pixlen, pixo]], [cent+[-pixlen, -pixo]]])
    
    verts = numpy.append(vertsv, vertsh)
    faces = [0, 2, 1, 1, 2, 3, 4, 6, 5, 5, 6, 7]
    res = GLtriangleMessage(verts, faces)
    
    return res


def circleGLTM(r = 100, fcount = 75, proj=Projector(), crosshairs=True):
    fb = numpy.array([1,2])
    faces = []
    verts = proj.rcent()
    for i in xrange(fcount):
        th = numpy.pi*2*(i/(fcount-1.0))
        verts = numpy.append(verts, proj.rcent() + (r*numpy.array([numpy.cos(th), numpy.sin(th)])))
    for i in xrange(fcount-1):
        faces = numpy.append(faces, [0])
        faces = numpy.append(faces, (i)+fb)
    
 
 
    res = GLtriangleMessage(numpy.int_(verts), numpy.int_(faces))

    b = crosshairGLTM()
    b.setColors([255,0,0])

    if crosshairs: gltm = GLtmAppend(b,res )
    else: gltm = res


    return gltm


def axisGLTM(o=200, s = 30, proj=Projector()):

    #triangle is +X, square is +Y
    cent = proj.rcent()
    vertsX = numpy.array([[cent+[o,0]], [cent+ [(o+2*s),s] ], [cent+ [(o+2*s),-s] ]])
    
    vertsY = numpy.array([    [cent+[-s, o]] , [cent+[s, o]] , [cent+[s, o+2*s]] ,  [cent+[-s, o+2*s]] ])
    
    verts = numpy.append(vertsX, vertsY)
    faces = [0, 2, 1, 3, 4, 5, 3, 5, 6]
    res = GLtriangleMessage(verts, faces)

    c = crosshairGLTM(pixlen=o/2)
    
    gltm = GLtmAppend(c, res)
    return gltm

#def circleGLTM(radius, centerpos, normal, virtual, PROJ=[[848, 480], [0.73, .33]]):
    


#returns a GLtriangleMessage based on a virtual target and 'line segments' object
def segProjections(vt, segs, proj=Projector(), animcount = 10, color=[0,255,0]):
    
    virtual = robot.Target(vt[0], vt[1])
    
    if ((virtual.__class__.__name__ == 'Target') & validSegments(segs)):         

        #blank GLtriangleMessage
        gltm = GLtriangleMessage()
        glArrows = GLtriangleMessage()

        #counting line segments
        segcount = numpy.shape(segs.vecs)[0]

        #explicitly loop through the line segments, and find pixel locations for segments
        for i in xrange(segcount):       

            #endpoints are segs.verts[i] and segs.verts[i+1]
            #vector is seg.vecs[i]

            cVec = unitv(segs.vecs[i])
            transL = cVec*segs.thick
            transA = cVec*(segs.thick+segs.athick)

            

            pBase = [proj.pixloc(segs.verts[i], virtual), proj.pixloc(segs.verts[i+1], virtual)]
            pExtrude = [proj.pixloc((segs.verts[i]+transL), virtual), proj.pixloc((segs.verts[i+1]+transL), virtual)]
            pArrow = [proj.pixloc((segs.verts[i]+transA+transL), virtual), proj.pixloc((segs.verts[i+1]+transA+transL), virtual)]

            glArrows = GLtmAppend(glArrows, arrowPos(numpy.vstack((pExtrude, pArrow)), animcount))
            
            gltm.verts = numpy.append(gltm.verts, numpy.append(pBase,pExtrude))
            
            face0 = [0,2,1, 2,1,3]; betweenface = [1,3,6]
            ifaces = numpy.array(face0) + i*4; ibfaces = numpy.array(betweenface) + i*4 

            if (i < (segcount - 1)): faceout = numpy.append(ifaces,ibfaces)
            else: faceout = ifaces

            gltm.faces = numpy.append(gltm.faces, faceout)
            
        gltm = GLtmAppend(gltm,glArrows)
        gltm.setColors(color)
        
        return gltm
            
#takes 4 bounding points, and returns GLtm with arrows
def arrowPos(pix, acount):
    #pix: [pExtrude0, pExtrude1, pArrow0, pArrow1]

    athick = .2
    pix = numpy.array(pix)

    pEmid = (pix[0]+pix[1])/2
    pAmid = (pix[2]+pix[3])/2

    vMid = (pEmid - pAmid) 
    v0 = (pix[0]-pix[2]) 
    v1 = (pix[1]-pix[3])

    arrow0 = numpy.array([pix[2], (pix[2] + athick*v0), (pAmid+(athick*vMid)), (pAmid+(athick*2*vMid)), pix[3], (pix[3] + athick*v0)])
    arrowIncrement = numpy.array([v0, v0, vMid, vMid, v1, v1])
    bounds = numpy.array([pix[0], pix[0], pEmid, pEmid, pix[1], pix[1]])
    verts = arrow0.flatten(); 

    for j in xrange(1, acount-1):
        i = (float(j)/acount)
        cpos = arrow0 + i*arrowIncrement 
        inbounds = ((cpos- bounds) >= 0) 
        verts = numpy.vstack((verts, cpos.flatten()))    
    faces = [0, 1, 2, 1, 3, 2, 2, 3, 5, 2, 5, 4]
    
    res = GLtriangleMessage(verts, faces)
    return res

def norm(v):
    vec = numpy.array(v).flatten()[0:3]
    return numpy.sqrt(numpy.dot(vec,vec))

def unitvs(vs):
    return numpy.float_(vs)/numpy.sum(vs)

def unitv(v):
    v = numpy.array(v).flatten()[0:3]
    n = norm(v)
    if (n<>0): return v/norm(v)
    else: return v

def parallel(v1,v2):
    PTOL = 1e-2
    if (abs(norm(v1)-norm(v2)) < PTOL): return True
    else: return False

def vang(v1,v2):
    return numpy.arccos(numpy.dot(v1,v2))


def rotby(xy, ang, c=[0,0]):
    r = [[numpy.cos(ang), -numpy.sin(ang)], [numpy.sin(ang), numpy.cos(ang)]]
    res = numpy.dot(xy, r)
    res += c
    return res

def virtual(worktarget=[945,0,0], cartesians=[945,0,1375]):
    # returns a virtual target that the robot can actually reach
    # virtual target will be pointed towards worktarget and located at cartesians
  
    # Quaternion [0,0,1,0] has z vector [0,0,-1]
    z0 = [0,0,-1]; zq0 = [0,0,1,0]; a = [[0],[0],[1],[1]];
    if (len(cartesians) == 3) & (len(worktarget) == 3):
        

        #unit direction vector from defined robot position to worktarget position
        dvec = unitv((worktarget-numpy.array(cartesians)))

        if (~parallel(dvec,z0)):
            rotvec = numpy.cross(dvec, z0); 
            rotangle = numpy.arcsin(norm(rotvec)/(norm(z0)*norm(dvec)))
            rotquat = transformations.quaternion_about_axis(-rotangle, rotvec)
            virtualresult = [cartesians, transformations.quaternion_multiply(rotquat, zq0)]
        else: virtualresult = [cartesians, zq0]

        return virtualresult
    else: return False

# this function takes a list of 2D verticies, and then projects them onto a plane, specified by three points in the robot coordinates. 
def projectPlane(flat=[[0,10],[10,0]], plane=[[ 1460.,-870.,-221.05733113], [ 1460.,700.,-223.52123963], [ 1820.,0., -224.42066831]]):
    mheight = numpy.mean(numpy.array(plane)[:,2])

    #skip the rotation, the plane is close enough
    tiled = numpy.tile(mheight, (numpy.shape(flat)[0],1))
    return numpy.hstack((flat, tiled))

    plane = unitvs(plane)
    if numpy.shape(plane) <> (3,3): return 0
    n2 = [0,0,1]; n3 = [1,0,0] 

    plane = numpy.array(plane)
    n1 = unitv(numpy.cross((plane[1]-plane[0]), (plane[2]-plane[0])))
    x1 = plane[0]; x2 = [0,0,0]; x3 = [0,0,0]
   
    alpha = vang(n1, n2)
    if alpha > (math.pi/2): alpha = alpha - math.pi

    #print 'alpha: ', alpha
    ns = numpy.vstack((n1, n2, n3))
    ns = numpy.rot90(ns, 3); ns = numpy.fliplr(ns)
    nsD = numpy.linalg.det(ns)
    
    if nsD <> 0: nsDI = 1/nsD
    else: return  numpy.hstack((flat,numpy.zeros((numpy.shape(flat)[0],1))))
    
    xz = nsDI*numpy.float_(numpy.dot(x1,n1)*numpy.cross(n2,n3)+numpy.dot(x1,n2)*numpy.cross(n3,n1)+numpy.dot(x3,n3)*numpy.cross(n1,n2))
    tm = transformations.rotation_matrix(alpha, n1, xz)

    flatlen = numpy.shape(flat)[0]
    flat = numpy.hstack((flat, numpy.zeros((flatlen,1)), numpy.ones((flatlen,1))))
    for i in range(len(flat)):
        flat[i] = numpy.dot(tm, flat[i])
    flat[:,2] += mheight
    return flat[:,0:3]
    


def flatfile_to_robot(virtual, filename='verts.txt', rot=math.radians(90) , trans=[1000,0], p=Projector()):
    verts = numpy.loadtxt(open(filename, 'r'), delimiter=',')
    verts *= 25.4
    verts = rotby(verts, rot, trans)
    robotcoordinates = projectPlane(verts)
    return robotcoordinates

#inputs a file name, outputs a GLTM for solid quadrilaterals 
def QuadsFile(virtual, rot=math.radians(90) , trans=[1000,0], filename='verts.txt', p=Projector()):
    a = numpy.loadtxt(open(filename, 'r'), delimiter=',')
    a *= 25.4
    return projectQuads(a, virtual, rot=rot, trans=trans, p=p)


def projectQuads(verts, virtual, rot=math.radians(90) , trans=[1000,0], p=Projector()):
    verts = rotby(verts, rot, trans)

   # print 'verts: ', verts
    rc = projectPlane(verts)
    #print 'rc: ', rc
    pc = p.pluralpixloc(rc, virtual)
    
    gface = []; 
    baseface = numpy.array([0,1,2,0,3,2,0,3,1,1,2,3])
    for i in range(int(numpy.shape(pc)[0]/4)):
        gface = numpy.append(gface, (baseface+(i*4)))
    

    a = GLtriangleMessage(numpy.array(pc).flatten(), gface)
    return a
