# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
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

bno = BNO08X_I2C(i2c, reset=reset_pin, address= addr, debug=False)
#bno = BNO08X_I2C(i2c)
bno.begin_calibration()
#time.sleep(5)
#bno.enable_feature(BNO_REPORT_ACCELEROMETER)
#bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
#bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
start_time = time.monotonic()
calibration_good_at = None
while True:
    time.sleep(0.1)

    print("Magnetometer:")
    mag_x, mag_y, mag_z = bno.magnetic
    print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    print("")

    print("Game Rotation Vector Quaternion:")
    (
        game_quat_i,
        game_quat_j,
        game_quat_k,
        game_quat_real,
    ) = bno.game_quaternion
    print(
        "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f"
        % (game_quat_i, game_quat_j, game_quat_k, game_quat_real)
    )
    calibration_status = bno.calibration_status
    print(
        "Magnetometer Calibration quality:",
        adafruit_bno08x.REPORT_ACCURACY_STATUS[calibration_status],
        " (%d)" % calibration_status,
    )
    if not calibration_good_at and calibration_status >= 2:
        calibration_good_at = time.monotonic()
    if calibration_good_at and (time.monotonic() - calibration_good_at > 5.0):
        input_str = input("\n\nEnter S to save or anything else to continue: ")
        if input_str.strip().lower() == "s":
            bno.save_calibration_data()
            break
        calibration_good_at = None
    print("**************************************************************")

print("calibration done")
while True:
    time.sleep(0.2) 
    """
    print("Acceleration:")
    accel_x, accel_y, accel_z = bno.acceleration
    print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    print("")

    print("Gyro:")
    gyro_x, gyro_y, gyro_z = bno.gyro
    print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
    print("")

    print("Magnetometer:")
    mag_x, mag_y, mag_z = bno.magnetic
    print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    print("")

    print("Rotation Vector Quaternion:")
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print("I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real))
    print("")
    """
    #print("Rotation Vector Game Quaternion:")
    game_quat_i, game_quat_j, game_quat_k, game_quat_real = bno.game_quaternion
    #print("I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (game_quat_i, game_quat_j, game_quat_k, game_quat_real))
    #print("")

    print("Attitude:")
    roll, pitch, yaw = find_attitude(game_quat_real, game_quat_i, game_quat_j, game_quat_k)
    print("Roll: %0.6f  Pitch: %0.6f Yaw: %0.6f" % (roll, pitch, yaw))
    print("")

    print("Heading (dgr):")
    heading = round(yaw * 180/pi)
    print("heading: %0.0f°" % (heading))
    print("")
