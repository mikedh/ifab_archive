'''
Michael Dawson-Haggerty
CMU Robotics Institute
For Dr. David Bourne/iFAB

autocalibrate.py:
Automatically finds orientation and offset of a laser projector, using a checkerboard pattern, vision, and trigonometry 

Prep: Checkerboard on table, with corners established in robot coordinates. Fixed camera takes an image, and then the table/checkerboard is covered in white paper
1: Move projector into rough, down- facing position, put white paper on table, and record the checkerboard corners in robot coordinates
2: Move projector vertically, maintaining same orientation, and record checkerboard corners in robot coordinates
3: Based on change in x,y, and height we find vector to rotate around, and magnitude of the angle. Change quaternions to reflect this rotation
4: GOTO step 1, and if XY change is below threshold continue to step 5, otherwise rotate again until threshold is met
5: Orientation is established, now calculate XY offset based on checkerboard position relative to reported robot position
6: Go low height to high height, in steps, recording checkerboard corners at each height. This will be used to calculate Z offset, and to do a vector to pixel mapping
7: Save information to a tool file, and a projector calibration file. I think these will be switched to JSON from pickle

usage: python autocalibrate.py
'''

import transformations 
import robot, projector 
import numpy, time, math, cPickle, sys, os, json

trunk = os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]

location = [1500,0,1000]
quats = [0,0,1,0]
heightRange = [200,1500]





class calibrateProjector():
    def __init__(self, calibOrigin=[1500,0,1000], zRange=[200,1400], R=None, P=None, C=None):
        if R == None: self.R = robot.Robot()
        if P == None: self.P = projector.Projector()
        if C == None: self.C = visionCorners('cameraInfo.json')
        self.calibOrigin = calibOrigin
        self.zRange = zRange


    def run(self, saveas='tool.object'):
        q = self.findOrientation()
        self.R.getTool()


    #execute steps 1:4, returns properly oriented quaternions
    def findOrientation(self, quats = [.57358,0,.81915,0]):
        chessDimensions = self.P.ccbGLTM(box=100, crosshairs=False, send=True)

        self.R.setJoints([0,0,20,180,0,-180])
        self.R.setCartesian([[1016, 0, 1152], quats])
    
        #error threshold is 3mm/1000mm
        ethresh = .003; error = 1e9
        for attempt in xrange(3):
            XYH = getXYH(quats)
            error = numpy.arcsin(numpy.sqrt(XYH[0]**2+XYH[1]**2)/XYH[2])
            if error < ethresh: break
            quats = xyhOrientation(XYH, quats)
        return quats


    def getXYH(self):
        xy = numpy.array(self.calibOrigin)[0:2]
        for z in self.zRange:
            self.R.setCartesian(numpy.append(xy, z))
            lowC= self.cornerAxis(chessDimensions)
        
            self.R.setCartesian(numpy.append(xy, z))
            highC= cornerAxis(chessDimensions)
        XYH = numpy.append((highC-lowC)[0:2], numpy.diff(zRange))
        return XYH


#Mabaran, this class outline is all yours :-D
class visionCorners():
    def __init__(self, cameraInfo):    
        try: 
            self.cameraInfo = json.load(open(cameraInfo, 'r'))
            #connect to camera, etc

        except: print 'camera init failed'

    def getCorners(self, chessDimensions):
        #returns an array with chessDimensions[0]*chessDimensions[1] XYZ points, based on cameraInfo data structure, of corners in robot coordinates
        return numpy.zeros(chessDimensions)

    def cornerAxis(self, chessDimensions):
        return numpy.average(self.getCorners(chessDimensions), axis=0)
        


def calcToolRot(currentTool, currentQuats):
    b = transformations.quaternion_multiply([0,0,1,0], currentQuats)
    toolquat = transformations.quaternion_multiply(currentTool[1],b) 
    toolquat = transformations.quaternion_inverse(toolquat)
    tool = [currentTool[0], toolquat]
    return tool

def calcToolCart(currentTool, currentQuats, adjustmentCart):
        #adjustmentCart is XYZ
    
    tstep = numpy.float_(adjustmentCart)
    if len(tstep) == 3: tstep = numpy.append(tstep, 1)
    else: return False
    
    T = transformations.quaternion_matrix(currentTool[1])
    Tinv = numpy.linalg.inv(T)
    P = transformations.quaternion_matrix(currentQuats)
    BCP = numpy.dot(P, Tinv)
    print 'BCP: ', BCP
    
    toolRes = currentTool
    tsprime = numpy.dot(tstep,BCP)
    toolRes[0] += numpy.array(tsprime[0:3])
    
    return toolRes

def xyhOrientation(dxyz, currentQuats):
    # (x,y) values are based on starting the projector low, if high-low reverse signs
    # quats is quaternions of robot orientation
    # this requires tool be set with orientation component [1,0,0,0]

    dx = float(dxyx[0]); dy = float(dxyz[1]) dh = float(dxyz[2])

    c = math.sqrt((dx*dx) + (dy*dy))
    thr = math.atan(c/dh)
    qrotation = transformations.quaternion_about_axis(-thr, [dy, -dx ,0])
    
    print 'current quats: ', quats, ' qrotation: ', qrotation
    if (~(dx==0) & ~(dy==0)):
        resq = transformations.quaternion_multiply(qrotation, currentQuats)
        return resq
    else: 
        print "no rotation, zero values"
        return False
        

if __name__ == '__main__':
    c = calibrateProjector()
    c.run(saveas='tool.object.projector')
