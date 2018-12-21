
import ue9
import sys
from time import sleep

# Open the U3
d = ue9.UE9()

def sensorDoor(daq, doorval):


    print doorval

    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 2,  TimerClockBase = 4, TimerClockDivisor = 15, Timer0Mode = 0, Timer1Mode = 0, Timer0Value = 60600, Timer1Value =doorval)
    sleep(1)
    daq.timerCounter(UpdateConfig = True, NumTimersEnabled = 0)
    


if __name__ == '__main__':
    doorlist = [60000, 62050] #[open, closed]
    
    for i in [0,1]:
        sensorDoor(d, doorlist[i])
        sleep(3)
    
d.close

