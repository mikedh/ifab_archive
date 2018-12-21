#Iout, into SHUNT_RESISTOR for voltage conversion
AIN0_1402_MEASUREMENT = 0

#Switch output for error state from 1402
AIN1_1402_ERROR = 1

#set HIGH to turn off 1402, LOW to turn on sensor
FIO2_1402_POWER = 6002
FIO2_1402_INTYPE = 6102


import ue9
import sys, time, numpy
from collections import deque

timeQ = deque([], maxlen=10)
timeQ.append(time.time())

#Hz
updateFreq = 10

updatePer = 1/float(updateFreq)

#ohms
SHUNT_RESISTOR = 226.8

#V
ERROR_CUTOFF = 0.852 

# Open the U9 DAQ
try: 
    d = ue9.UE9()
    print "connected via USB"
except: 
    d = ue9.UE9(ethernet=True, ipAddress="192.168.125.209")
    print "connected over ethernet"

#turn on laser sensor
#d.writeRegister(FIO2_1402_INTYPE, 1)
d.writeRegister(FIO2_1402_POWER, 0)



def sensorDoor(daq, doorstate=0):
    doorval = [62050,60000]

    doorindex = int((doorstate>0))

    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 2,  TimerClockBase = 4, TimerClockDivisor = 15, Timer0Mode = 0, Timer1Mode = 0, Timer0Value = 60600, Timer1Value =doorval[doorindex])
    time.sleep(1)
    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 0)


def volt2distance(voltage, resistor):
    # v = ir, i = v/r
    iout = voltage / resistor
    dist = (iout-.004)*(200/.016)
    return dist


def close():
    #turn off 1402 & close DAQ 
    d.writeRegister(FIO2_1402_POWER, 1)
    sensorDoor(d, 0)
    #d.writeRegister(FIO2_1402_INTYPE, 0)
    d.close

try: 
    sensorDoor(d, 1)
    while (True):
        timeQ.append(time.time())
        v = d.readRegister(AIN0_1402_MEASUREMENT)
        e = d.readRegister(AIN1_1402_ERROR)
        dist = volt2distance(v, SHUNT_RESISTOR)
        extime = numpy.mean(numpy.diff(timeQ))
        print 'head distance:', volt2distance(v, SHUNT_RESISTOR), 'mm\terror:', (e), '\tupdating at', format((1/extime), '04.2f'), 'Hz'

    
    
        time.sleep(updatePer)
        
except: pass
finally:
    close()



