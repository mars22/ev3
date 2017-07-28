#!/usr/bin/env python3

from time import sleep

import ev3dev.fonts as fonts
from ev3dev.ev3 import *

lcd = Screen()


def lcd_print(text):
    lcd.clear()
    lcd.draw.text((40, 50), text, font=fonts.load('helvB24'))
    lcd.update()

lcd_print('Start')
sleep(2)
lcd_print('Exit')
sleep(5)
