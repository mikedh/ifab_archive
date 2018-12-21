
import ue9
import sys
from time import sleep

# Open the U3
d = ue9.UE9()


servoON = '-s' in sys.argv

so = '-so' in sys.argv
sc = '-sc' in sys.argv

do = '-do' in sys.argv
dc = '-dc' in sys.argv


initialize = '-i' in sys.argv
flipP = '-p' in sys.argv

maxV = 63000
minV = 58000

r = maxV-minV

maxV = int(minV + r*.52)
steps = 1




def projectorToggle(daq):
    daq.writeRegister(6001, 1)
    sleep(1.5)
    daq.writeRegister(6001, 0)

def projectorDoor(daq, doorstate=0):
    doorval = [60600, 58000] #[closed, open]
    doorindex = int(int(doorstate)>0)

    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 1,  TimerClockBase = 4, TimerClockDivisor = 15,  Timer0Mode = 0, Timer0Value = doorval[doorindex])
    sleep(.5)
    print doorval[doorindex]
    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 0)


def sensorDoor(daq, doorstate=0):
    doorval = [61950, 60250] #[open, closed]
    doorindex = int((doorstate>0))

    print doorval[doorindex]

    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 2,  TimerClockBase = 4, TimerClockDivisor = 15, Timer0Mode = 0, Timer1Mode = 0, Timer0Value = 60600, Timer1Value =doorval[doorindex])
    sleep(1)
    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 0)




if flipP: projectorToggle(d)

if so: projectorDoor(d, 1)
if sc: projectorDoor(d, 0)

if do: sensorDoor(d, 1)
if dc: sensorDoor(d, 0)
    
if initialize: 
    projectorToggle(d)
    
    projectorDoor(d, 1)
    
d.close

