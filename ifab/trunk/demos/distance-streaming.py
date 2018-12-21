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
ERROR_CUTOFF = 0.852 


import ue9
from time import sleep
from datetime import datetime
import struct, sys
import traceback
import numpy

ethernet = '-e' in sys.argv



def forwardKinematics(robotCart, robotQuat, distance):
    # returns a point in space based on robot position, orientation, and measured distance.
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

def volt2distance(voltage, resistor=SHUNT_RESISTOR):
    # v = ir, i = v/r
    iout = voltage / resistor
    dist = (iout-.004)*(200/.016)
    return dist




# MAX_REQUESTS is the number of packets to be read.
MAX_REQUESTS = 20

# At 96 Hz or higher frequencies, the number of samples will be MAX_REQUESTS times 8 (packets per request) times 16 (samples per packet).
# Currently over ethernet packets per request is 1.

if not ethernet: 
    try: 
        d = ue9.UE9() #ethernet=True, ipAddress="192.168.125.209") 
        print 'connected via USB'
    except:
        print 'connection via USB failed'
        ethernet = True

if ethernet: 
    try: 
        d = ue9.UE9(ethernet=True, ipAddress="192.168.125.209") 
        print 'connected via ethernet'
    except: print 'connection failed via ethernet'


# For applying the proper calibration to readings.
d.getCalibrationData()

print "configuring UE9 stream"

d.streamConfig( NumChannels = 4, ChannelNumbers = [ AIN0_1402_MEASUREMENT, AIN1_1402_ERROR, EIO_FIO, MIO_CIO], ChannelOptions = [ 0,0,0,0 ], SettlingTime = 0, Resolution = 12, SampleFrequency = 1500 )


#turn on laser sensor
d.writeRegister(FIO4_1402_POWER, 0)


try:
    start = datetime.now()
    print "start stream", start
    d.streamStart()
    
    missed = 0
    dataCount = 0
    byteCount = 0
    start = datetime.now()
    
    for r in d.streamData():
        if r is not None:
            # Our stop condition
            if dataCount > MAX_REQUESTS:
                break
            
            if r['errors'] != 0:
                print "Error: %s ; " % r['errors'], datetime.now()

            if r['numPackets'] != d.packetsPerRequest:
                print "----- UNDERFLOW : %s : " % r['numPackets'], datetime.now()

            if r['missed'] != 0:
                missed += r['missed']
                print "+++ Missed ", r['missed']

            # Comment out this print and do something with r
            #print len(r['AIN0']), 'data: ', r['AIN0'], '\n'
            
            #print r['AIN194']
            bunchcount = len(r['AIN193'])
            #print (r['AIN193'])

            c = numpy.zeros(bunchcount)
            
            pbs = 2100
            print r['AIN1'][0]
            for i in xrange(bunchcount):
                bs = ((bin(r['AIN193'][i][1])[3:] + (bin(r['AIN194'][i][0])[2:]).zfill(4)),2)

    
                if (bs <> pbs):
                    val = volt2distance(r['AIN0'][i])
                    err = r['AIN1'][i]
                    #print 'error',  err
                pbs =  bs
                

            #t = bs[0:9] + bs[10] + bs[9]
                
            #c[i] =  int(t,2)#int((bin(r['AIN193'][0][1])[3:] + (bin(r['AIN194'][0][0])[2:]).zfill(4)),2)
            #print int((bin(r['AIN193'][0][1])[3:] + (bin(r['AIN194'][0][0])[2:]).zfill(4)),2)
            #print c

            dataCount += 1
            
        else:
            # Got no data back from our read.
            # This only happens if your stream isn't faster than the 
            # the USB read timeout, ~1 sec.
            print "No data", datetime.now()
except:
    print "".join(i for i in traceback.format_exc())
finally:
    stop = datetime.now()
    print "stream stopped."
    d.streamStop()
    d.writeRegister(FIO4_1402_POWER, 1)
    d.close()

    total = dataCount * d.packetsPerRequest * d.streamSamplesPerPacket
    print "%s requests with %s packets per request with %s samples per packet = %s samples total." % ( dataCount, d.packetsPerRequest, d.streamSamplesPerPacket, total )
    print "%s samples were lost due to errors." % missed
    total -= missed
    print "Adjusted number of samples = %s" % total
    
    runTime = (stop-start).seconds + float((stop-start).microseconds)/1000000
    print "The experiment took %s seconds." % runTime
    print "%s samples / %s seconds = %s Hz" % ( total, runTime, float(total)/runTime )
