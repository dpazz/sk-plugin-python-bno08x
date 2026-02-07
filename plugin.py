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

import sys, json, datetime, logging, os, time;

import ujson, requests

from urllib.request import urlopen

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
    #print("I2C devices found: ", [hex(i) for i in devices], file = sys.stderr)
    for i in devices:
        logger.info ("I2C devices found: [" + hex(i) + "]")
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

def internet_on():
    try:
        response = urlopen('https://www.google.com/', timeout=10)
        return True
    except: 
        return False

def getSignalkVariation():
    try:
        # use the last value stored in signalk
        resp = requests.get('http://localhost:3000/signalk/v1/api/vessels/self/navigation/magneticVariation/$source', verify=False)
        data = ujson.loads(resp.content)
        if data != my_source :
            try:
                resp = requests.get('http://localhost:3000/signalk/v1/api/vessels/self/navigation/magneticVariation/values', verify=False)
                data = ujson.loads(resp.content)
                return data[my_source]['value']
            except:
                return decl_rad # anyway return the last available value stored in 'decl_rad' global
        else:
            resp = requests.get('http://localhost:3000/signalk/v1/api/vessels/self/navigation/magneticVariation/value', verify=False)
            data = ujson.loads(resp.content)
            return data
    except:
        return decl_rad # anyway return the last available value stored in 'decl_rad' global

def getDeclination():
    resp = requests.get('http://localhost:3000/signalk/v1/api/vessels/self/navigation/position/value', verify=False)
    data = ujson.loads(resp.content)
    #TODO Manage eception 'position' not available in Signalk data
    lat = "{:.4f}".format(data['latitude'])
    lon = "{:.4f}".format(data['longitude'])
    if internet_on() :
        NOAA_DeclCalcAPI = "https://www.ngdc.noaa.gov/geomag-web/calculators/calculateDeclination?lat1="\
+lat+"&lon1="+lon+"&key=zNEw7&resultFormat=json"
        resp = requests.get(NOAA_DeclCalcAPI, verify=True)
        try:
            # manage malformed/unexpected resp content
            data = ujson.loads(resp.content)
            decl_res = data['result']
            for key in decl_res:
                #logger.info("Declination got from NoAA")
                return key['declination'] * pi/180 # NoAA conventionally responds in degrees
        except:
            ret = getSignalkVariation()
            return ret # anyway return last available value
    else:
        ret = getSignalkVariation()
        return ret # anyway return last available value 

class pluginConfig():
    def __init__(self, dev, rate, rd, nc, nd, di, de, ohdg, odev, oroll, opitch):
        self.name = dev
        self.rate = rate
        self.delay = rd
        self.calib_needed = nc
        self.hdgOffset = ohdg
        self.hdgDeviation = odev
        self.rollOffset = oroll
        self.pitchOffset = opitch
        self.decl_needed = nd
        self.decl_interval = di
        self.decl_estimate = de
        self.delaycount = 0

class CustomAdapter(logging.LoggerAdapter):
    """
    From Python cookbook:
        This example adapter expects the passed in dict-like object to have a
        'pluginid' key, whose value in brackets is prepended to the log message.
    """
    def process(self, msg, kwargs):
        return '[%s] %s' % (self.extra['pluginid'], msg), kwargs

def skOutput(mySource, path, value):

    skData = {'updates': [{'source': {'label': 'IMU sensor', 'src': mySource }, 'timestamp': datetime.datetime.utcnow().isoformat() + "Z", 'values': [{'path': path, 'value': value}]}]}
    print(json.dumps(skData) + '\n')

def skOutput_att(mySource, path, r, p, y):

    skData = {'updates': [{'source': {'label': 'IMU sensor', 'src': mySource }, 'timestamp': datetime.datetime.utcnow().isoformat() + "Z", 'values': [{'path': path, 'value':\
            {"pitch": p, "roll": r, "yaw": y}}]}]}
    print(json.dumps(skData) + '\n')

def sensorReportLoop(mySource,rate, bno, dCfg):
    times_for_calib_status_update = 100 # calibration status sent every 100 times the normal attitude delta is sent
    declInterval_start = time.monotonic()
    global decl_rad
    while True:
        time.sleep(rate)
        if dCfg.delaycount == 0:
            dCfg.delaycount = dCfg.delay
            with open ('debug.log', 'a') as sys.stdout :
                game_quat_i, game_quat_j, game_quat_k, game_quat_real = bno.game_quaternion
                sys.stdout.flush()
            sys.stdout = sys.__stdout__ # restore normal stdout file object
            roll, pitch, yaw = find_attitude(game_quat_real, game_quat_i, game_quat_j, game_quat_k)
            roll += dCfg.rollOffset * pi/180
            pitch += dCfg.pitchOffset * pi/180
            yaw += dCfg.hdgOffset * pi/180
            skOutput_att(mySource,'navigation.attitude', roll, pitch, yaw)
            skOutput(mySource,'navigation.headingCompass', yaw) # headingCompass from 0 to 2*pi radians clockwise
            headingMagnetic = yaw + dCfg.hdgDeviation * pi/180
            #TODO 1: implement a 'deviation table' of values for different bearings and interpolate between them
            skOutput(mySource,'navigation.headingMagnetic', headingMagnetic)
            if dCfg.decl_needed :
                skOutput ( mySource, 'navigation.headingTrue', headingMagnetic + decl_rad )
                time_current = time.monotonic()
                if ((time_current - declInterval_start) >= dCfg.decl_interval*3600) : # Interval in hours
                    decl_rad = getDeclination()
                    declInterval_start = time_current
                    skOutput ( mySource, 'navigation.magneticVariation', decl_rad )
            if dCfg.calib_needed :
                if times_for_calib_status_update == 0:
                    times_for_calib_status_update = 100
                    if os.stat('debug.log').st_size > 1000000 : # save only last (of less than 1mb size) debug.log
                        mode = 'w'
                    else:
                        mode = 'a'
                    with open('debug.log', mode) as sys.stdout : # redirect stdout to debug.log to avoid debug messages 
                                                                 # being read by javascript parent process
                        print ("DEBUG: PERIODIC CALIBRATION AT "+ datetime.datetime.utcnow().isoformat())
                        calibration_status = bno.calibration_status
                        sys.stdout.flush()
                    sys.stdout = sys.__stdout__ # restore normal stdout file object
                    skOutput(mySource,'sensors.magnetometer.calibration_status', calibration_status)
                    skOutput(mySource,'sensors.magnetometer.calibration_quality', adafruit_bno08x.REPORT_ACCURACY_STATUS[calibration_status])
                else:
                    times_for_calib_status_update -=1
            sys.stdout.flush()
        else:
            dCfg.delaycount -= 1

def sensorCalibrate(dev, mySource,  bno):
    with open ('calibration.log', 'w') as sys.stdout: # log all packet error during calibration 
        bno.begin_calibration()
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        calibration_good = False
        calibration_good_at = None
        start_time = time.monotonic()
        source ='BNO08X_I2C_AT[' +hex(dev)+']'
        print ("=============== "+ source + " CALIBRATION START =========================")
        print ("")
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
            if calibration_status < 2:
                calibration_good = False
                calibration_good_at = None
            if not calibration_good and calibration_status >= 2:
                calibration_good_at = time.monotonic()
                calibration_good = True
            current_time = time.monotonic()
            if calibration_good and (current_time - calibration_good_at > 5.0):
                # wait 5 seconds to let calib being stabilized
                break
            if (current_time - start_time) > 50.0 :
                logger.critical (' Calibration timeout !!!')
                raise ValueError (' CALIBRATION TIMEOUT ERROR')
        print ("Calibrate obtained in "+ repr(current_time-start_time) + ' fractional sec.')
        print ('Calibration status = ' + repr(calibration_status))
        print ('Calibration accuracy: ' + adafruit_bno08x.REPORT_ACCURACY_STATUS[calibration_status])
        print ("=============== "+ source + " CALIBRATION END =========================")
        bno.save_calibration_data()
        sys.stdout.flush()
    sys.stdout = sys.__stdout__ # restore normal stdout behavior
    skOutput(mySource,'sensors.magnetometer.calibration_status',calibration_status)
    skOutput(mySource,'sensors.magnetometer.calibration_quality',adafruit_bno08x.REPORT_ACCURACY_STATUS[calibration_status])
    sys.stdout.flush()
    logger.info("calibration done")

    
# main entry (enable logging and check device configurations)

myConfigList: list[pluginConfig] = []


basic_logger = logging.getLogger(__name__)
logger = CustomAdapter(basic_logger, {'pluginid': "sk-py-bno08x"})
logging.basicConfig(stream = sys.stderr, level = logging.DEBUG)
logging.getLogger("adafruit_bno08x").setLevel(logging.WARNING)

# Source - https://stackoverflow.com/a/11029841
# Posted by aknuds1, modified by community. See post 'Timeline' for change history
# Retrieved 2026-02-02, License - CC BY-SA 3.0

logging.getLogger("requests").setLevel(logging.WARNING)
logging.getLogger("urllib3").setLevel(logging.WARNING)

config = json.loads(input())

for options in config["imuDevices"]:

    plgCfg = pluginConfig(options["devName"],
                          options["devRefresh"],
                          options["devDelayReports"],
                          options["devCalibRequired"],
                          options["devDeclRequired"],
                          options["devDeclInterval"],
                          options["devDeclEstimate"],
                          options["devHdgOffset"],
                          options["devHdgDeviation"],
                          options["devRollOffset"],
                          options["devPitchOffset"])
    myConfigList.append(plgCfg)
    
    global my_source
    global my_source_addr_part
    global decl_rad

    i2c = busio.I2C(board.SCL, board.SDA)
    try:
        addr = scan_for_bno(i2c)
    except ValueError as e:
        logger.critical(e)
    #
    if addr != plgCfg.name :
        logger.critical("THE CONFIGURED ADDRESS VALUE '" + hex[plgCfg.name] + "'" +" IS DIFFERENT FROM THE ONE FOUND --> '" + hex[addr] + "'")
    package_name = 'sk-py-bno08x'
    my_source_addr_part = 'I2C_at['+hex(addr)+']'
    my_source = package_name + '.' + my_source_addr_part
    rRate = 1/plgCfg.rate # convert reports/sec in secs btw reports
    bno = BNO08X_I2C(i2c, reset=None , address= addr, debug=False)
    if plgCfg.calib_needed :
        sensorCalibrate(addr, my_source_addr_part, bno)
    else :
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
    
    sys.stdout.flush() #to guarantee that output buffer is clean after bno calib/enable features
    time.sleep(0.2)
    
    # set the estimate defined in schema as the initial default value
    decl_rad = plgCfg.decl_estimate * pi/180 # choosen to use degrees for user input in config
    if plgCfg.decl_needed :
        decl_rad = getDeclination()
        skOutput( my_source_addr_part, 'navigation.magneticVariation', decl_rad )
        logger.info ("navigation.magneticVariation initialized")

    sensorReportLoop(my_source_addr_part, rRate, bno, plgCfg)    

for line in iter(sys.stdin.readline, b''):
    try:
        data = json.loads(line)
        sys.stderr.write(json.dumps(data))
    except:
        sys.stderr.write('error parsing json\n')
        sys.stderr.write(line)
