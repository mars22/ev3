#!/usr/bin/env python3

import threading
from time import sleep

import rpyc

conn = rpyc.classic.connect('ev3dev.local')
ev3 = conn.modules['ev3dev.ev3']  # import ev3dev.ev3 remotely
# from  ev3dev import ev3

# fonts = conn.modules['ev3dev.ev3.fonts']      # import ev3dev.ev3 remotely

lcd = ev3.Screen()


def check(condition, message):
  """
  Check that condition is true,
  loudly complain and throw an exception otherwise.
  """
  if not condition:
    ev3.Sound.speak(message).wait()
    raise Exception(message)


def lcd_print(text):
  lcd.clear()
  lcd.draw.text((40, 50), text)
  lcd.update()


def move(motor: ev3.LargeMotor, speed: int):
  motor.run_forever(speed_sp=speed)


class PID:

  MAX_DUTY_CYCLE = 100

  def __init__(self, set_point, kp=1.0, ki=1.0, kd=0.0):
    self.set_point = set_point
    self.kd = kd
    self.ki = ki
    self.kp = kp

    self.integral = 0.0  # sum of all errors from start till that moment
    self.prev_error = 0.0

  def update(self, set_point: int, current_val: int):
    current_error = set_point - current_val
    self.integral += current_error
    derivative = self.prev_error - current_error
    self.prev_error = current_error

    duty_cycle = (current_error * self.kp) + (self.integral * self.ki) + (derivative * self.kd)
    return sorted([-self.MAX_DUTY_CYCLE, duty_cycle, self.MAX_DUTY_CYCLE])[1]


class Odometry:

  def __enter__(self):
    self.flag = threading.Event()
    self.flag.set()
    return self

  def __exit__(self, *args):
    self.flag.clear()

  def print_odometry(self, left_motor, right_motor):
    def worker(cond: threading.Event):
      while cond.is_set():
        error = left_motor.position - right_motor.position
        print('Left: {left} -- Right: {right} -- error: {error}'.format(
          left=left_motor.position, right=right_motor.position, error=error)
        )
        sleep(0.20)

    t = threading.Thread(target=worker, args=(self.flag,))
    t.start()

  def run_forward_odometry(self, left_motor: ev3.LargeMotor, right_motor: ev3.LargeMotor, speed: int):
    def normalize(num):
      return sorted([-100, num, 100])[1]

    def worker(cond: threading.Event):
      right_motor.position = left_motor.position = 0
      left_motor.run_direct(duty_cycle_sp=speed)
      right_motor.run_direct(duty_cycle_sp=speed)
      pid = PID(set_point=0, kp=2.8, ki=0.2)
      while cond.is_set():
        error = left_motor.position - right_motor.position
        # print('Left: {left} -- Right: {right} -- error: {error}'.format(
        #   left=left_motor.position, right=right_motor.position, error=error)
        # )
        print('Left - Right: {error}'.format(error=error))
        nv = (pid.update(0, error)/10)
        print('NV: {error}'.format(error=nv))
        if error > 0:
          # lewe za szybkie
          left_motor.duty_cycle_sp = normalize(speed + nv)
        elif error < 0:
          #prawe za wolno
          # right_motor.duty_cycle_sp = normalize(speed + abs(nv))
          right_motor.duty_cycle_sp = normalize(speed - nv)
        else:
          right_motor.duty_cycle_sp = speed
          left_motor.duty_cycle_sp = speed


    t = threading.Thread(target=worker, args=(self.flag,))
    t.start()

  def run_backward_odometry(self, left_motor: ev3.LargeMotor, right_motor: ev3.LargeMotor, speed: int):
    def normalize(num):
      if -100 < num < 100:
        return num
      elif num > 100:
        return 100
      else:
        return -100

    def worker(cond: threading.Event):
      left_motor.run_direct(duty_cycle_sp=speed)
      right_motor.run_direct(duty_cycle_sp=speed)
      while cond.is_set():
        error = abs(left_motor.position) - abs(right_motor.position)
        print('Left: {left} -- Right: {right} -- error: {error}'.format(
          left=left_motor.position, right=right_motor.position, error=error)
        )
        l_error = speed+error
        r_error = speed-error
        if error > 0:
          right_motor.duty_cycle_sp = normalize(r_error)
        else:
          left_motor.duty_cycle_sp = normalize(l_error)

    t = threading.Thread(target=worker, args=(self.flag,))
    t.start()


class RobotMotors:
  def __init__(self):
    self.speed_sp = 300
    self.all_motors = [
      ev3.LargeMotor(address)
      for address in (ev3.OUTPUT_B, ev3.OUTPUT_C)
    ]
    self.left_motor, self.right_motor = self.all_motors

    check(self.left_motor.connected,
          'Connect a Left motor to {port} port'.format(port=ev3.OUTPUT_B))
    check(self.right_motor.connected,
          'Connect a Right motor to {port} port'.format(port=ev3.OUTPUT_C))
    self.reset_motors()

  def reset_motors(self):
    for motor in self.all_motors:
      motor.reset()
      motor.position = 0
      motor.stop_action = 'brake'

  def stop(self, motor: ev3.Motor):
    motor.stop()
    # to make extra sure the motors have stopped:
    motor.run_forever(speed_sp=0)

  def stop_motors(self):
    for motor in self.all_motors:
      self.stop(motor)

  def forward(self):
    for motor in self.all_motors:
      motor.run_forever(speed_sp=self.speed_sp)

  def backward(self):
    for motor in self.all_motors:
      motor.run_forever(speed_sp=-self.speed_sp)

  def turn_left(self):
    self.left_motor.run_to_rel_pos(position_sp=-180, speed_sp=self.speed_sp)
    self.right_motor.run_to_rel_pos(position_sp=180, speed_sp=self.speed_sp)
    self.left_motor.wait_while('running')
    self.right_motor.wait_while('running')

  def turn_right(self):
    self.left_motor.run_to_rel_pos(position_sp=180, speed_sp=self.speed_sp)
    self.right_motor.run_to_rel_pos(position_sp=-180, speed_sp=self.speed_sp)
    self.left_motor.wait_while('running')
    self.right_motor.wait_while('running')

  def demo(self):
    ev3.Sound.speak('I am ready').wait()
    with Odometry() as od:
      od.print_odometry(*self.all_motors)
      self.forward()
      sleep(2)
      self.backward()
      sleep(2)
      self.turn_left()
      self.turn_right()
      self.turn_right()
      self.turn_left()
      self.stop_motors()
      ev3.Sound.speak('That is all folks').wait()

  def demo_run(self):
    # ev3.Sound.speak('I am ready').wait()
    with Odometry() as od:
      od.run_forward_odometry(*self.all_motors, 30)
      sleep(10)
      self.stop_motors()
      # ev3.Sound.speak('Done').wait()


# # Connect ultrasonic sensor to any sensor port
# # and check it is connected.
# us = UltrasonicSensor()
# assert us.connected, "Connect a US sensor to any sensor port"
# # Put the US sensor into distance mode.
# us.mode = 'US-DIST-CM'
#
# lcd_print('Init')

if __name__ == '__main__':
  lcd_print('Start')

  RobotMotors().demo_run()

  sleep(5)  # when running from Brickman, need time to admire image
