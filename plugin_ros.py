"""
MIT License

Copyright (c) 2025 arancino1

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


import sys
import json
import time
import math
import serial
import logging
import threading
import datetime

import numpy as np

from array import array


# .. global variables

CmdPacket_Begin = 0x49
CmdPacket_End = 0x4D
CmdPacketMaxDatSizeRx = 73

i = 0
CS = 0
RxIndex = 0
cmdLen = 0

buf = bytearray(5 + CmdPacketMaxDatSizeRx)


class devConfig():
    def __init__(self, dev, rd, ohdg, oroll, opitch):
        self.name = dev
        self.delay = rd
        self.hdgOffset = ohdg
        self.rollOffset = oroll
        self.pitchOffset = opitch
        self.dcount = 0


def skOutput(dev, path, value):

    skData = {'updates': [{'source': {'label': 'ROS sensor', 'src': dev}, 'timestamp': datetime.datetime.utcnow().isoformat() + "Z", 'values': [{'path': path, 'value': value}]}]}
    sys.stdout.write(json.dumps(skData) + '\n')


def Cmd_RxUnpack(dCfg, buf, DLen):

    if buf[0] == 0x11:
        ctl = (buf[2] << 8) | buf[1]

        L = 7

        if ((ctl & 0x0001) != 0): L += 6
        if ((ctl & 0x0002) != 0): L += 6
        if ((ctl & 0x0004) != 0): L += 6

        if ((ctl & 0x0008) != 0):
            scaleMag = 0.15106201171875

            tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L + 0]) * scaleMag
            tmpY = np.short((np.short(buf[L + 3]) << 8) | buf[L + 2]) * scaleMag
            #tmpZ = np.short((np.short(buf[L + 5]) << 8) | buf[L + 4]) * scaleMag

            L += 6

            magHdg = math.atan2(tmpY, tmpX) * (180.0 / math.pi)

            if (magHdg < 0):
                magHdg = abs(magHdg)
            else:
                magHdg = 360.0 - magHdg

            magHdg += dCfg.hdgOffset

            if (magHdg >= 360.0):
                magHdg -= 360.0
            if (magHdg < 0):
                magHdg += 360.0

        if ((ctl & 0x0010) != 0): L += 8
        if ((ctl & 0x0020) != 0): L += 8

        if ((ctl & 0x0040) != 0):
            scaleAngle = 0.0054931640625

            tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L + 0]) * scaleAngle
            tmpY = np.short((np.short(buf[L + 3]) << 8) | buf[L + 2]) * scaleAngle
            tmpZ = np.short((np.short(buf[L + 5]) << 8) | buf[L + 4]) * scaleAngle

            L += 6

            roll = 0
            pitch = -tmpY

            if (tmpX < 0):
                roll = -180.0 - tmpX
            else:
                roll = 180.0 - tmpX

            roll += dCfg.rollOffset
            pitch += dCfg.pitchOffset

        magDev = 0.0

        if dCfg.dcount == 0:
            dCfg.dcount = dCfg.delay

            skOutput(dCfg.name, 'navigation.headingCompass', magHdg * math.pi / 180.0)
            skOutput(dCfg.name, 'navigation.magneticDeviation', magDev)
            skOutput(dCfg.name, 'navigation.headingMagnetic', magHdg * math.pi / 180.0)

            skOutput(dCfg.name, 'navigation.attitude.yaw', 0.0)
            skOutput(dCfg.name, 'navigation.attitude.roll', roll * math.pi / 180.0)
            skOutput(dCfg.name, 'navigation.attitude.pitch', pitch * math.pi / 180.0)

            sys.stdout.flush()

        else:
            dCfg.dcount = dCfg.dcount - 1



def Cmd_GetPkt(dCfg, byte):

    global CS, i, RxIndex, buf, cmdLen

    CS += byte

    if RxIndex == 0:
        if byte == CmdPacket_Begin:
            i = 0
            buf[i] = CmdPacket_Begin
            i += 1
            CS = 0
            RxIndex = 1

    elif RxIndex == 1:
        buf[i] = byte
        i += 1
        if byte == 255:
            RxIndex = 0
        else:
            RxIndex += 1

    elif RxIndex == 2:
        buf[i] = byte
        i += 1
        if byte > CmdPacketMaxDatSizeRx or byte == 0:
            RxIndex = 0
        else:
            RxIndex += 1
            cmdLen = byte

    elif RxIndex == 3:
        buf[i] = byte
        i += 1
        if i >= cmdLen + 3:
            RxIndex += 1
    elif RxIndex == 4:
        CS -= byte
        if (CS & 0xFF) == byte:
            buf[i] = byte
            i += 1
            RxIndex += 1
        else:
            RxIndex = 0
    elif RxIndex == 5:
        RxIndex = 0
        if byte == CmdPacket_End:
            buf[i] = byte
            i += 1
            hex_string = " " . join(f"{b:02X}" for b in buf[0:i])
            Cmd_RxUnpack(dCfg, buf[3:i-2], i-5)
            return 1
    else:
        RxIndex = 0
    return 0


def Cmd_PackAndTx(dev, pDat, DLen):
    if DLen == 0 or DLen > 19:
        return -1

    buf = bytearray([0x00] * 46) + bytearray([0x00, 0xff, 0x00, 0xff, 0x49, 0xFF, DLen]) + bytearray(pDat[:DLen])

    CS = sum(buf[51:51 + DLen + 2]) & 0xFF
    buf.append(CS)
    buf.append(0x4D)

    dev.write(buf)
    return 0


def sensor_data_loop(dev, rate, dCfg):

    isCompassOn = 1
    barometerFilter = 2

    Cmd_ReportTag = 0x48

    params = bytearray([0x00 for i in range(0,11)])

    params[0] = 0x12
    params[1] = 5                   # Stationary state acceleration threshold
    params[2] = 255                 # Static zero return speed (unit cm/s) 0: No return to zero 255: Return to zero immediately
    params[3] = 0                   # Dynamic zero return speed (unit cm/s) 0: No return to zero
    params[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1);
    params[5] = int(rate)           # transmission frame rate of data actively reported [value 0-250HZ], 0 means 0.5HZ
    params[6] = 1                   # Gyroscope filter coefficient [value 0-2], the larger the value, the more stable it is but the worse the real-time performance.
    params[7] = 3                   # Accelerometer filter coefficient [value 0-4], the larger the value, the more stable it is but the worse the real-time performance.
    params[8] = 5                   # Magnetometer filter coefficient [value 0-9], the larger the value, the more stable it is but the worse the real-time performance.
    params[9] = Cmd_ReportTag & 0xff
    params[10] = (Cmd_ReportTag >> 8) & 0xff

    Cmd_PackAndTx(dev, params, len(params))
    time.sleep(0.2)

    # wake up sensor
    Cmd_PackAndTx(dev, [0x03], 1)
    time.sleep(0.2)

    # enable proactive reporting
    Cmd_PackAndTx(dev, [0x19], 1)

    # start magnetometer calibration
    #Cmd_PackAndTx([0x32], 1)
    #time.sleep(15.0)

    # end magnetometer calibration
    #Cmd_PackAndTx([0x04], 1)

    # Z-axis angle reset to zero
    #Cmd_PackAndTx([0x05], 1)
    #time.sleep(0.5)

    while True:
        data = dev.read(1)
        if len(data) > 0:
            Cmd_GetPkt(dCfg, data[0])


# main entry (enable logging and check device configurations)

myDevList: list[devConfig] = []

logging.basicConfig(stream = sys.stderr, level = logging.DEBUG)

config = json.loads(input())

for desc in config["rosDevices"]:

    rDelay = 0
    rRate = desc["devRefresh"]

    if rRate <= 0:
        rDelay = abs(rRate)
        rRate = 1

    devCfg = devConfig(desc["devName"], rDelay, desc["devHdgOffset"], desc["devRollOffset"], desc["devPitchOffset"])
    myDevList.append(devCfg)

    ser = serial.Serial(devCfg.name, 115200, timeout = 2)

    if ser.isOpen():
        threading.Timer(0.1, sensor_data_loop, [ser, rRate, devCfg]).start()
    else:
        logging.critical("ERROR opening: '" + devCfg.name + "'")

for line in iter(sys.stdin.readline, b''):
    try:
        data = json.loads(line)
        sys.stderr.write(json.dumps(data))
    except:
        sys.stderr.write('error parsing json\n')
        sys.stderr.write(line)