"""
MIT License

Copyright (c) 2026 dpazz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""

import sys, json, threading;
from math import atan2, asin, pi, sqrt

import board
import busio

from digitalio import DigitalInOut

import adafruit_bno08x
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

from micropython import const

_BNO08X_DEFAULT_ADDRESS = const(0x4A)
_BNO08X_ALTERNATIVE_ADDRESS = const(0x4B)

def scan_for_bno(i2c):

    while not i2c.try_lock():
      pass

    # Scan and print results
    #print("I2C Scanner")
    devices = i2c.scan()
    print("I2C devices found: ", [hex(i) for i in devices])
    if _BNO08X_ALTERNATIVE_ADDRESS in devices:
      address=_BNO08X_ALTERNATIVE_ADDRESS
    elif _BNO08X_DEFAULT_ADDRESS in devices:
      address=_BNO08X_DEFAULT_ADDRESS
    else:
      i2c.unlock # Anyway unlock the bus
      raise ValueError("NO VALID BNO08X ADDRESS FOUND IN THE I2C BUS")

    # Unlock the bus
    i2c.unlock()
    return address

def find_attitude(dqw, dqx, dqy, dqz):
    norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
    dqw = dqw / norm
    dqx = dqx / norm
    dqy = dqy / norm
    dqz = dqz / norm

    ysqr = dqy * dqy
    xsqr = dqx * dqx
    zsqr = dqz * dqz

    t0 = +2.0 * (dqw * dqx + dqy * dqz)
    t1 = +1.0 - 2.0 * (xsqr + ysqr)
    roll_raw = atan2(t0, t1)
    roll = -roll_raw # report clockwise in radians from 0 to + or - pi

    t2 = +2.0 * (dqw * dqy - dqz * dqx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_raw = asin(t2)
    pitch = -pitch_raw # report clockwise in radians from 0 to + or - pi

    t3 = +2.0 * (dqw * dqz + dqx * dqy)
    t4 = +1.0 - 2.0 * (ysqr + zsqr)
    yaw_raw = atan2(t3, t4)
    if yaw_raw > 0:
        yaw = 2*pi - yaw_raw
    else:
        yaw = abs(yaw_raw)
    # heading in radians clockwise from 0 to 2*pi

    return roll, pitch, yaw 

i2c = busio.I2C(board.SCL, board.SDA)
reset_pin = DigitalInOut(board.D7)
#
try:
    addr = scan_for_bno(i2c)
except ValueError as e:
  print(e)
#

n = 0
def outputSk():
    global n
    skData = {'updates': [{ 'values': [{'path': 'some.path', 'value': n}]}]}
    sys.stdout.write(json.dumps(skData) + '\n' + json.dumps(skData))
    sys.stdout.write('\n')
    sys.stdout.flush()
    n += 1
    threading.Timer(1.0, outputSk).start()

threading.Timer(1.0, outputSk).start()

for line in iter(sys.stdin.readline, b''):
    try:
        data = json.loads(line)
        sys.stderr.write(json.dumps(data))
    except:
        sys.stderr.write('error parsing json\n')
        sys.stderr.write(line)

