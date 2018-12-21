'''
Michael Dawson-Haggerty
CMU Robotics Institute
For David Bourne/iFAB


robot.py: contains classes and support functions which interact with an ABB Robot running our software stack (RAPID code modules LOGGER and SERVER)
'''


import numpy, socket, asyncore, transformations, os.path, sys, cPickle, thread, threading, time
from multiprocessing import Process, Lock, Value, Queue
import jackDistance
    
class Robot:
    def __init__(self, IP='192.168.125.1', PORT=5000, wobj=[[0,0,0],[1,0,0,0]], tool=[[0,0,0], [1,0,0,0]], speed = (350,150), toolfile = False, TFNAME = 'tool.object.valid', zeroJoints = False, verbose=False):
        
        self.BUFLEN = 4096 
        self.v = verbose

        if verbose: print 'starting robot'
        try: 
            self.robsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.robsock.connect((IP, PORT))
            if verbose: print 'robot socket connected'
        except:
            print 'socket connection to robot failed, exiting'
        self.setTool()
        if toolfile: setToolFile(TFNAME)
        self.setWorkObject(wobj)
        self.setSpeed(speed)
        self.setZone('z5')
    
        if zeroJoints: self.setJoints()

    def goTarget(self, targ):
        if (targ.__class__.__name__ == 'Target'):
            if (targ.valid()):
                self.setCartesian([targ.cart, targ.quat])
            
    def setSpeed(self, speed=(350,50)):
        speed = numpy.array(speed).flatten()
        if (numpy.shape(speed)[0] == 2):
            msg = "08 000 " 
            msg = msg + format(speed[0], "+08.1f") + " " + format(speed[1], "+08.2f") + " #"  
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            if self.v: print 'setSpeed:', speed
            return data
        else: return False

    def setWorkObject(self, wobj=[[0,0,0],[1,0,0,0]]):
        #wobj is [[cartesians], [quaternions]]
        if self.checkCoordinates(wobj):
            msg = "07 000 " 
            msg = msg + format(wobj[0][0], "+08.1f") + " " + format(wobj[0][1], "+08.1f") + " " + format(wobj[0][2], "+08.1f") + " " 
            msg = msg + format(wobj[1][0], "+08.5f") + " " + format(wobj[1][1], "+08.5f") + " " 
            msg = msg + format(wobj[1][2], "+08.5f") + " " + format(wobj[1][3], "+08.5f") + " #"    
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            if self.v: print 'setWO:', data
            return data
        else: return False

    def setCartesian(self, pos):
        #pos is [[cartesians], [quaternions]]
        if self.v: print 'setCart:', pos
        if len(pos) == 7: pos = [pos[0:3], pos[3:7]]
        if self.checkCoordinates(pos):
            msg = "01 000 " 
            msg = msg + format(pos[0][0], "+08.1f") + " " + format(pos[0][1], "+08.1f") + " " + format(pos[0][2], "+08.1f") + " " 
            msg = msg + format(pos[1][0], "+08.5f") + " " + format(pos[1][1], "+08.5f") + " " 
            msg = msg + format(pos[1][2], "+08.5f") + " " + format(pos[1][3], "+08.5f") + " #"    
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            return data
        else:
            print 'coordinate check failed'
            return False

    def circularSingle(self, circlePoint, endPoint):
        #*Point is [[cartesians], [quaternions]]
        if self.v: print 'setCircular:', endPoint
        if self.checkCoordinates(circlePoint) & self.checkCoordinates(endPoint):
            msg = "35 000 " 
            msg = msg + format(circlePoint[0][0], "+08.1f") + " " + format(circlePoint[0][1], "+08.1f") + " " + format(circlePoint[0][2], "+08.1f") + " " 
            msg = msg + format(circlePoint[1][0], "+08.5f") + " " + format(circlePoint[1][1], "+08.5f") + " " 
            msg = msg + format(circlePoint[1][2], "+08.5f") + " " + format(circlePoint[1][3], "+08.5f") + " #"    

            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)

            msg = "36 000 " 
            msg = msg + format(endPoint[0][0], "+08.1f") + " " + format(endPoint[0][1], "+08.1f") + " " + format(endPoint[0][2], "+08.1f") + " " 
            msg = msg + format(endPoint[1][0], "+08.5f") + " " + format(endPoint[1][1], "+08.5f") + " " 
            msg = msg + format(endPoint[1][2], "+08.5f") + " " + format(endPoint[1][3], "+08.5f") + " #"    

            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)

            return data
        else:
            print 'coordinate check failed'
            return False


    def circularList(self, posList):
        #posList is (n,3)

        #0: start
        #1: circ
        #2: end/start
        #3: circ

        s = numpy.shape(posList)
        
        if (len(s) <> 2): return False
        if (s[1] <> 2): return False
        if ((s[0] < 3) | ((s[0]-3)%2 <> 0)): return False

        if len(posList[0][0]) <> 3: return False
        if len(posList[0][1]) <> 4: return False

        self.setCartesian(posList[0])
        for i in xrange(1, s[0]-1, 2):
            data = self.circularSingle(posList[i], posList[i+1])
        return data
    

    def setTable(self, height):
        #our table goes from -550 to +40
        msg = "12 000 "
        msg = msg + format(height, "+08.1f") + " #"
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        return data


    # takes a much nicer (n,7) in the place of the dumb sequence ordering. I want to switch everything to this format 
    def setBufferMod(self, posList, gotoFirst=False):
        s = numpy.shape(posList)
        
        if (len(s) <> 2): return False
        if (s[1] <> 7): return False 

        self.clearBuffer()
        if self.lenBuffer() <> 0: return False

        if gotoFirst: self.setCartesian( [posList[0][0:3], posList[0][4:7]])

        for i in posList: self.addBuffer([i[0:3], i[3:7]])
        if self.lenBuffer() == s[0]: return True
        else: 
            self.clearBuffer()
            return False


    def setBuffer(self, posList, gotoFirst=False):
        #posList is (n,2)
        #adds every position in posList to the buffer
        s = numpy.shape(posList)
        
        if (len(s) <> 2): return False
        if (s[1] <> 2): return False 

        self.clearBuffer()
        if self.lenBuffer() <> 0: return False

        if gotoFirst: self.setCartesian(posList[0])

        for i in posList: self.addBuffer(i)
        if self.lenBuffer() == len(posList): return True
        else: 
            self.clearBuffer()
            return False

    def addBuffer(self, pos):
        #pos is [[cartesians], [quaternions]]
        #appends single position to the buffer
        if self.checkCoordinates(pos):
            msg = "21 000 " 
            msg = msg + format(pos[0][0], "+08.1f") + " " + format(pos[0][1], "+08.1f") + " " + format(pos[0][2], "+08.1f") + " " 
            msg = msg + format(pos[1][0], "+08.5f") + " " + format(pos[1][1], "+08.5f") + " " 
            msg = msg + format(pos[1][2], "+08.5f") + " " + format(pos[1][3], "+08.5f") + " #"    
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            if self.v: print 'addBuffer:', data
            time.sleep(.1)
            return data
        else:
            if self.v: print 'coordinate check failed'
            return False

    def executeBuffer(self):
        #execute every move in buffer as MoveL command (linear move)
        msg = "30 000 #"
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        return data

    def weldBuffer(self):
        #execute every move in buffer as ArcMoveL and ArcL (intermediate weld point)
        msg = "31 000 #"
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        return data

    def clearBuffer(self):
        msg = "22 000 #"
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        return data

    def lenBuffer(self):
        msg = "23 000 #"
        self.robsock.send(msg)
        data = str(self.robsock.recv(self.BUFLEN)).split(' ')
        return int(float(data[4]))

    def setTool(self, tool=[[0,0,0], [1,0,0,0]]):
        #tool is [[cartesians], [quaternions]]
        if self.checkCoordinates(tool):
            msg = "06 000 " 
            msg = msg + format(tool[0][0], "+08.1f") + " " + format(tool[0][1], "+08.1f") + " " + format(tool[0][2], "+08.1f") + " " 
            msg = msg + format(tool[1][0], "+08.5f") + " " + format(tool[1][1], "+08.5f") + " " 
            msg = msg + format(tool[1][2], "+08.5f") + " " + format(tool[1][3], "+08.5f") + " #"    
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            self.tool = tool
            if self.v: print 'setTool:', msg
            return data
        else: return False

    def setJoints(self, joints=[0,0,0,0,90,0]):
        #joint positions are in degrees
        if ((numpy.shape(joints[0]) == ()) & (numpy.shape(joints) == (6,))):
            data = self.setSingleJ(joints)
            return data
        elif numpy.shape(joints[0]) == (6,):
            if self.v: print 'multimove'
            data = None
            for j in joints:
                if numpy.shape(j) == (6,): data = self.setSingleJ(j)
            return data
        else: return False

    def setSingleJ(self, j):
        if ((numpy.shape(j[0]) == ()) & (numpy.shape(j) == (6,))):
            msg = "02 000 " 
            msg = msg + format(j[0], "+08.2f") + " " + format(j[1], "+08.2f") + " " + format(j[2], "+08.2f") + " " 
            msg = msg + format(j[3], "+08.2f") + " " + format(j[4], "+08.2f") + " " + format(j[5], "+08.2f") + " #" 
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)  
            return data
        else: return False

    def setToolFile(self, filename, verbose=False):
        if os.path.exists(filename):
            f = open(filename, 'rb');        
            try: tool = cPickle.load(f)
            except: 
                print 'toolfile failed to load!'
                return False
        else: 
            print 'toolfile ', filename, 'doesn\'t exist'
            return False
        self.setTool(tool)
        if verbose: print 'setting tool', tool

    def getTool(self): 
        return self.tool

    def getJoints(self):
        msg = "04 000 #"
        self.robsock.send(msg)
        data = numpy.float_(str(self.robsock.recv(self.BUFLEN)).split(' '))
        return data[4:10]

    def getCartesian(self):
        msg = "03 000 #"
        self.robsock.send(msg)
        data = numpy.float_(str(self.robsock.recv(self.BUFLEN)).split(' '))
        return [data[4:7], data[7:11]]
        
    def setZone(self, zoneKey='z1', finep = False, manualZone=[]):
        zoneDict = {'z0': [.3,.3,.03], 'z1': [1,1,.1], 'z5': [5,8,.8], 'z10': [10,15,1.5], 'z15': [15,23,2.3], 'z20': [20,30,3], 'z30': [30,45,4.5], 'z50': [50,75,7.5], 'z100': [100,150,15], 'z200': [200,300,30]}

        #zoneKey: uses values from RAPID handbook (stored here in zoneDict), 'z*' YOU SHOULD USE THESE
        #finep: go to point exactly, and stop briefly before moving on

        #manualZone = [pzone_tcp, pzone_ori, zone_ori]
        #pzone_tcp: mm, radius from goal where robot tool center is not rigidly constrained
        #pzone_ori: mm, radius from goal where robot tool orientation is not rigidly constrained
        #zone_ori: degrees, zone size for the tool reorientation

        if finep: zone = [0,0,0]
        else:
            if len(manualZone) == 3: zone = manualZone
            elif zoneKey in zoneDict.keys(): zone = zoneDict[zoneKey]
            else: return False 
        msg = "09 000 " 
        msg = msg + str(int(finep)) + " "
        msg = msg + format(zone[0], "+08.4f") + " " + format(zone[1], "+08.4f") + " " + format(zone[2], "+08.4f") + " #" 
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        if self.v: print 'setZone key', zoneKey
        return data



    def checkCoordinates(self, coords):
        if (len(coords) == 2):
            if ((len(coords[0]) == 3) & (len(coords[1]) == 4)):
                return True
        return False

    def projectModel(self, model): 
        return False

    def __del__(self):
        self.robsock.shutdown(socket.SHUT_RDWR)
        self.robsock.close()



'''
logger object opens connection to robot, which starts position data streaming over the network at 30 Hz + jitter
Opening a connection starts the loop on the robot which then begins outputting sync signal (11 bit number which counts up, then loops over) which labjack can stream at the same rate as the displacement sensor

instantiation of a logger object will turn on the displacement sensor as well and read values from that. 

An example which would move the robot through positions in robotTestTargets (an array structured like [ [[xyz],[quat]], ...]
Then a point cloud would be recorded from those passes. 

R = Robot()
L = Logger()

L.start()
for i in robotTestTargets:
     R.setTarget(i)
PointCloud = L.stop()
'''


class Logger:
    def __init__(self, IP='192.168.125.1', PORT=5111, verbose=False):
        
        self.IP = IP
        self.PORT = PORT
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.v = verbose

        self.netQ = Queue()
        self.jackQ = Queue()
        self.resultQ = Queue()

        self.Sync = jackDistance.Sync(self.netQ, self.jackQ, self.resultQ, verbose)

        if self.v: print 'starting net read process'
        pn = threading.Thread(target=self.getNet).start()
        
        if self.v: print 'starting the labjack thread'
        pj = threading.Thread(target=jackDistance.readDistance, args=(self.jackQ, self.Sync.active, verbose,)).start()

        if self.v: print 'starting net/labjack sync thread'
        ps = threading.Thread(target=self.Sync.syncLoop).start()
  
        time.sleep

    def start(self):
        self.Sync.start()

    def dump(self, sequence=False):
        dumpCount = int(resultQ.qsize())
        for i in xrange(dumpCount):
            if self.resultQ.empty(): return False
            if starting: result = self.resultQ.get(); starting=False
            else: result = numpy.vstack((result, self.resultQ.get()))
        return jackDistance.forwardKinematicsList(result, sequence)

    def stop(self, sequence=False):
        self.Sync.stop()
        result = []; starting = True
        time.sleep(1)

        while True:
            if self.resultQ.empty(): 
                return jackDistance.forwardKinematicsList(result, sequence)
            if starting: result = self.resultQ.get(); starting=False
            else: result = numpy.vstack((result, self.resultQ.get()))
                    
    def close(self):
        self.Sync.active.value = False
        time.sleep(2)

    def getNet(self):
        self.s.connect((self.IP, self.PORT))
        self.s.setblocking(1)
        
        while True:
            if not self.Sync.active.value: break
            data = self.s.recv(8192)
            a = str(data).split(' ')
            if len(a) == 10: 
                self.Sync.nQ.put([int(a[2]), numpy.float_(a[3:10])])

        #supposedly not necessary but the robot gets mad if you don't do this
        self.s.shutdown(socket.SHUT_RDWR)
        self.s.close()
        if self.v: print "network stream terminated"
    

class Target:
    # Target object represents position/orientation on a part
    
    def __init__(self, cartesians=[], quaternions=[]):
        self.cart = cartesians
        self.quat = quaternions
        
    def valid(self):
        if ((len(self.cart) == 3) & ((len(self.quat) == 4))):
            return True
        else:
            return False

    def disp(self):
        return str([list(self.cart), list(self.quat)])

