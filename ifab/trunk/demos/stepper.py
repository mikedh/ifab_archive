
import ue9
import sys, time

# Open the U3
d = ue9.UE9()





FIO3_STATE_REGISTER = 6003
d.writeRegister(FIO3_STATE_REGISTER, 1)
time.sleep(1)
d.writeRegister(FIO3_STATE_REGISTER, 0)
print 'motor enabled'

time.sleep(1)

d.timerCounter(UpdateConfig = True, NumTimersEnabled = 2,  TimerClockBase = 3, TimerClockDivisor = 256, Timer0Mode = 7, Timer0Value=256, Timer1Mode = 9, Timer1Value = 5 )
time.sleep(5)


d.timerCounter(UpdateConfig = True, NumTimersEnabled = 0)



d.writeRegister(FIO3_STATE_REGISTER, 1)

    
d.close

