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

import sys, json, threading, datetime, logging, time;
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
    print("I2C devices found: ", [hex(i) for i in devices], file = sys.stderr)
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
#deltacount = 0
class pluginConfig():
    def __init__(self, dev, rate, rd, nc, ohdg, odev, oroll, opitch):
        self.name = dev
        self.rate = rate
        self.delay = rd
        self.calib_needed = nc
        self.hdgOffset = ohdg
        self.hdgDeviation = odev
        self.rollOffset = oroll
        self.pitchOffset = opitch
        self.delaycount = 0

def skOutput(dev, path, value):

    skData = {'updates': [{'source': {'label': 'IMU sensor', 'src': 'BNO08X_at['+hex[dev]+']'}, 'timestamp': datetime.datetime.utcnow().isoformat() + "Z", 'values': [{'path': path, 'value': value}]}]}
    sys.stdout.write(json.dumps(skData) + '\n')

def sensorReportLoop(dev,rate, bno, dCfg):

    if dCfg.delaycount == 0:
        dCfg.delaycount = dCfg.delay
        game_quat_i, game_quat_j, game_quat_k, game_quat_real = bno.game_quaternion
        roll, pitch, yaw = find_attitude(game_quat_real, game_quat_i, game_quat_j, game_quat_k)
        roll += dCfg.rollOffset * pi/180
        pitch += dCfg.pitchOffset * pi/180
        yaw += dCfg.hdgOffset * pi/180
        skOutput(dev,'navigation.attitude.roll',roll)
        skOutput(dev,'navigation.attitude.pitch',pitch)
        skOutput(dev,'navigation.attitude.yaw',yaw)
        #heading = round(yaw * 180/pi)
        skOutput(dev,'navigation.headingMagnetic',yaw)
        skOutput(dev,'navigation.headingCompass',yaw + dCfg.hdgDeviation * pi/180)

        sys.stdout.flush()
    else:
        dCfg.delaycount -= 1
    threading.Timer(rate, sensorReportLoop, [dev, rate, bno, dCfg]).start()

def sensorCalibrate(dev, bno):
    bno.begin_calibration()
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
    start_time = time.monotonic()
    calibration_good_at = None
    while True:
        time.sleep(0.1)
        mag_x, mag_y, mag_z = bno.magnetic
        (
            game_quat_i,
            game_quat_j,
            game_quat_k,
            game_quat_real,
        ) = bno.game_quaternion
        calibration_status = bno.calibration_status
        if not calibration_good_at and calibration_status >= 2:
            calibration_good_at = time.monotonic()
        if calibration_good_at and (time.monotonic() - calibration_good_at > 5.0):
            skOutput(dev,'sensor.magnetometer.calibration_status',calibration_status)
            skOutput(dev,'sensor.magnetometer.calibration_quality',adafruit_bno08x.REPORT_ACCURACY_STATUS[calibration_status])
            sys.stdout.flush()
            bno.save_calibration_data()
            break
        calibration_good_at = None
    logging.debug("calibration done")

    
# main entry (enable logging and check device configurations)

myConfigList: list[pluginConfig] = []

logging.basicConfig(stream = sys.stderr, level = logging.DEBUG)

config = json.loads(input())

for options in config["imuDevices"]:

    plgCfg = pluginConfig(options["devName"],
                          options["devRefresh"],
                          options["devDelayReports"],
                          options["devCalibRequired"],
                          options["devHdgOffset"],
                          options["devHdgDeviation"],
                          options["devRollOffset"],
                          options["devPitchOffset"])
    myConfigList.append(plgCfg)
    
    i2c = busio.I2C(board.SCL, board.SDA)
    reset_pin = DigitalInOut(board.D7)
    #
    try:
        addr = scan_for_bno(i2c)
    except ValueError as e:
        logging.critical(e)
    #
    if addr != plgCfg.name :
        logging.critical("THE CONFIGURED ADDRESS VALUE '" + hex[plgCfg.name] + "'" +" IS DIFFERENT FROM THE ONE FOUND --> '" + hex[addr] + "'")

    rRate = 1/plgCfg.rate # convert repots/sec in secs btw reports

    bno = BNO08X_I2C(i2c, reset=reset_pin, address= addr, debug=False)
    if plgCfg.calib_needed :
        sensorCalibrate(addr, bno)
    else :
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
    
    threading.Timer(0.2, sensorReportLoop, [addr, rRate, bno, plgCfg]).start()
        

for line in iter(sys.stdin.readline, b''):
    try:
        data = json.loads(line)
        sys.stderr.write(json.dumps(data))
    except:
        sys.stderr.write('error parsing json\n')
        sys.stderr.write(line)