'''
Michael Dawson-Haggerty
CMU Robotics Institute
For David Bourne/iFAB


jackDistance.py: contains functions which interact with the Labjack UE9 DAQ

UE9 Connectivity: 
Micro-Epsilon 1402-200: UE9 DB37 1:(Ground) 5:(Black On/Off) 18:(Brown, into 226 ohm shunt resistor, Error value) 37:(White, into 226 ohm shunt resistor, distance value) 
Servos: UE9 DB37 6:(Projector Door) 24:(1402-200 Door)
Syncronization Signal from ABB IRC5: UE9 DB15: 11 bits + ground

'''

import ue9


import struct,sys
import numpy
import thread,time
import transformations
from multiprocessing import Value, Queue, Lock


#does forward kinematics on an (n, 10) array (x/y/z, q0/qx/qy/qz, dist, error, sequence number)
def forwardKinematicsList(cloud, includeSequence=False):
    cs = numpy.shape(cloud)
    if len(cs) <> 2: return None
    if cs[1] <> 10: return None

    if includeSequence:
        resdim = (cs[0]-int(numpy.sum(numpy.rot90(cloud)[0])),4)
        res = numpy.zeros(resdim); rpos = 0 
        for i in xrange(cs[0]):
            if ((cloud[i][9])): pass # | (cloud[i][7] < 0)): pass
            else: 
                xyz = forwardKinematics(cloud[i][1:4], cloud[i][4:8], cloud[i][8])
                res[rpos] = numpy.append(xyz, cloud[i][0])

                if sum(numpy.abs(res[rpos])) == 0: print cloud[i]
                rpos += 1
        return res
    else:
        resdim = (cs[0]-int(numpy.sum(numpy.rot90(cloud)[0])),3)
        res = numpy.zeros(resdim); rpos = 0 
        for i in xrange(cs[0]):
            if ((cloud[i][9])): pass # | (cloud[i][7] < 0)): pass
            else: 
                res[rpos] = forwardKinematics(cloud[i][1:4], cloud[i][4:8], cloud[i][8])
                if sum(numpy.abs(res[rpos])) == 0: print cloud[i]
                rpos += 1
        return res
        

# returns a point in space based on robot position, orientation, and measured distance.
def forwardKinematics(robotCart, robotQuat, distance):
    a = [[0],[0],[1],[1]];
    dvec = unitv(numpy.dot(transformations.quaternion_matrix(robotQuat), a))
    result = robotCart + (dvec*distance)
    return result

def norm(v):
    vec = numpy.array(v).flatten()[0:3]
    return numpy.sqrt(numpy.dot(vec,vec))

def unitv(v):
    v = numpy.array(v).flatten()[0:3]
    n = norm(v)
    if (n<>0): return v/norm(v)
    else: return v

def parallel(v1,v2, PTOL = 1e-3):
    if (abs(norm(v1)-norm(v2)) < PTOL): return True
    else: return False

def volt2distance(voltage, resistor):
    # v = ir, i = v/r
    iout = voltage / resistor
    dist = (iout-.004)*(200/.016)
    return dist



#takes data from the labjack, and the network, and syncronizes them based on a common synchronization value. 
class Sync:
    def __init__(self, nQ, jQ, rQ, verbose=False):

        self.dJack = dict()
        self.dNet = dict()

        self.v = verbose

        self.nQ = nQ
        self.jQ = jQ
        self.rQ = rQ
        
        self.L = Lock()
        self.active = Value('i', True)
        self.record = Value('i', False)

    def start(self):
        self.dJack = dict()
        self.dNet = dict()

        self.record.value = True
        
    def stop(self):
        self.record.value = False
            
    def syncLoop(self):
        while self.active.value:
            if self.jQ.empty(): 
                time.sleep(.005)
            else:
                qpop = self.jQ.get()
                with self.L: self.addJack(qpop[0], qpop[1:])
            
            if self.nQ.empty(): 
                time.sleep(.005)
            else:
                qpop = self.nQ.get()
                with self.L: self.addNet(qpop[0], qpop[1:])
        if self.v: print 'exited sync loop'


    def addNet(self, indexNum, data):

        if self.record.value:
           
            inN = int(indexNum)
            if inN in self.dJack: 
                self.rQ.put(numpy.append(numpy.append(inN, data), self.dJack[inN]))
                #print 'recorded pt', inN, numpy.append(data, self.dJack[inN]) 
                del self.dJack[inN]
            #elif indexNum in self.dNet:     
                #pass
            else:
                self.dNet[inN] = data
                #print self.dNet.keys()

    def addJack(self, indexNum, data):
        if self.record.value:
           inN = int(indexNum)
           if inN in self.dNet: 
               self.rQ.put(numpy.append(numpy.append(inN,self.dNet[inN]),data))
               #print 'recorded pt', inN, numpy.append(self.dNet[inN],data)
               del self.dNet[inN]
           else:
               self.dJack[int(inN)] = data
               # print 'jackkeys', self.dJack.keys()

#opens/closes the 1402-200 door 
def sensorDoor(daq, doorstate=0):
    doorval = [62050,60000]
    doorindex = int((doorstate>0))

    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 2,  TimerClockBase = 4, TimerClockDivisor = 15, Timer0Mode = 0, Timer1Mode = 0, Timer0Value = 60600, Timer1Value =doorval[doorindex])
    time.sleep(1)
    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 0)

#streams distance, error, and sequence from the labjack at 1.5kHz
#when an edge is observed in the sequence value, it adds the information to a Sync object
#based partially on labjack example code, streaming-sample.py
def readDistance(jQ, active, verbose=False):
    try:

        #Iout, into SHUNT_RESISTOR for voltage conversion
        AIN0_1402_MEASUREMENT = 0

        #Switch output for error state from 1402
        AIN1_1402_ERROR = 1

        #set HIGH to turn off 1402, LOW to turn on sensor
        FIO4_1402_POWER = 6002

        #Read EIO,FIO,MIO,and CIO which have the sync signal encoded 
        EIO_FIO = 193	
        MIO_CIO = 194

        #ohms
        SHUNT_RESISTOR = 226.8
        #Volts
        ERROR_CUTOFF = 0.9 

        try: 
            d = ue9.UE9()
            if verbose: print 'connected to labjack locally over USB'
        except:
            print "ue9 failed to instantiate"
            sys.exit()
            #d = ue9.UE9(ethernet=True, ipAddress="192.168.125.209")
            #if verbose: print 'connected to labjack over ethernet'
        

        # For applying the proper calibration to readings.
        d.getCalibrationData()

        #print "configuring UE9 stream"
        d.streamConfig( NumChannels = 4, ChannelNumbers = [ AIN0_1402_MEASUREMENT, AIN1_1402_ERROR, EIO_FIO, MIO_CIO], ChannelOptions = [0,0,0,0], SettlingTime = 0, Resolution = 12, SampleFrequency = 1500 )

        pbs = 2100
        starting = True

        #open blast doors
        sensorDoor(d, 1)

        #turn on sensor
        d.writeRegister(FIO4_1402_POWER, 0)
        
        d.streamStart()

        missed = 0
        dataCount = 0
        byteCount = 0
        
        for r in d.streamData():
            if r is not None:
                if not active.value: break
                
                if r['errors'] != 0:
                    print "Error: %s ; " % r['errors']

                if r['numPackets'] != d.packetsPerRequest:
                    print "----- UNDERFLOW : %s : " % r['numPackets']

                if r['missed'] != 0:
                    missed += r['missed']
                    print "+++ Missed ", r['missed']
                
                if starting: bunchcount = len(r['AIN193'])
                c = numpy.zeros(bunchcount)
                
                for i in xrange(bunchcount):
                    bs = int((bin(r['AIN193'][i][1])[3:] + (bin(r['AIN194'][i][0])[2:]).zfill(4)),2)
                    if ((bs <> pbs) & (not starting)):
                        val = volt2distance(r['AIN0'][i], SHUNT_RESISTOR)
                        err = int( (r['AIN1'][i] > ERROR_CUTOFF) | (val<0))
                        jQ.put([bs,val,err])
                        
                    pbs = bs
                    if starting: starting = False
            else:
                # Got no data back from our read.
                # This only happens if your stream isn't faster than the 
                # the USB read timeout, ~1 sec.
                print "No data", datetime.now()
    except:
        print 'exception in jackDistance', sys.exc_info()[0]
    finally:
        d.streamStop()
        d.writeRegister(FIO4_1402_POWER, 1)
        sensorDoor(d, 0)
        d.close()
        if verbose: print "labjack cleanup complete."
