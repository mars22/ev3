#!/usr/bin/env python3
from ev3dev.ev3 import *

mB = LargeMotor('outB')
mC = LargeMotor('outC')

mB.stop()
mC.stop()

# to make extra sure the motors have stopped:
mB.run_forever(speed_sp=0)
mC.run_forever(speed_sp=0)