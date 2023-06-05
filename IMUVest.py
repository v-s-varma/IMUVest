from Quaternion import Quaternion
from collider import Collider
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from scipy.linalg import norm
import numpy
import socket
import math
import serial
import time


#Importing python libraries "***"DUNCAN's***""
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from multiprocessing import Process,Queue,Pipe
import serial
import time
from math import sin, cos, sqrt, atan2
import numpy as np
import sys
import time

#Importing Custom Functions ****DUNCAN's****
from gaitDetectClass import * #class gaitDetect, func testVal(shank gyZ, heel gyZ)
from packageHandlerFunc import * #package_handler_raw(tup)
from sensorClass import * #class sensorObject, newValues(valueArray), angularAccCalc(), angleCalc(gaitDetectObject)
from serialSend import * #ardno(msg as string)
from slipAlgorithmFunc import * #slipAlgorithm(pelvis_forward_acc, heel_forward_acc, L_hh)
from kneelingAlgorithm import * #kneelingDetection.kneelingDetection(objRT, objRS, objRH, objLT, objLS, objLH)
from CUNYreceiver import *
from NUCreceiver import *
from ARDUINOreceiver import *
from userinput import *
from variableDeclarations import *

from live_detect.live_dtw_detection import *
from live_detect.live_rms_detection import *
from live_detect.store_SrcSignal import *
from live_detect.store_MotherWavelet import *
from live_detect.live_cwt_detection import *


## MQTT imports
import time
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time


# communication functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("/leds/pi")
def on_message(client, userdata, msg):
    print(time.time())
    if msg.topic == '/leds/pi':
        if msg.payload ==b'D':
            print(time.time())
            print("Done")
def mqtt_comm_func():
    LED_PIN=24
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.output(LED_PIN, GPIO.LOW)
    client = mqtt.Client()
    client.onconnect = on_connect
    client.on_message = on_message
    client.username_pw_set("trg", "chordata")
    client.connect('localhost',1883,60)
    client.loop_start()
    print('MQTT is running, press Ctrl+C to quit...')
    #client.publish('/leds/esp8266',str(trip_duration))




class Limb():
    def __init__(self, name):
        self.length = 1
        self.start = (0, 0, 0)
        self.end = (0, 0, 0)
        self.orientation = None
        self.factor = None
        self.inverse = None
        self.name = name
        self.midpoint = None
        self.orthog = None
        self.gx = None
        self.gy = None
        self.gz = None
        self.ax = None
        self.ay = None
        self.az = None
        self.mx = None
        self.my = None
        self.mz = None

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        
        init_time=time.time()
        if(time.time()-init_time<10):
            
            self.gx = gx
            self.gy = gy
            self.gz = gz
            self.ax = ax
            self.ay = ay
            self.az = az
            self.mx = mx
            self.my = my
            self.mz = mz



        if self.factor != None:
            prev = Quaternion.hamilton_product(self.inverse, self.orientation)
            prev = sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz)
            self.orientation = Quaternion.hamilton_product(prev, self.factor)
        else:
            prev = Quaternion()
            for i in range(10):
                prev = sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz)
            conjugate = Quaternion.conjugate(prev)
            self.factor = Quaternion.hamilton_product(conjugate, 
                                                      self.orientation)
            self.orientation = Quaternion.hamilton_product(prev, self.factor)
            self.inverse = Quaternion.conjugate(self.factor)

    def to_string(self):
        #return "LENGTH:" + str(self.length) + "\tSTART:" + str(self.start) + \
        #       "\tEND" + str(self.end) + "\tQ:" + str(self.orientation) + \
        #       "\tFACTOR" + str(self.factor) + "\tINVERSE:" + str(self.inverse) + "\n"
        return str(self.gx) + "\t" + str(self.gy) + "\t" + str(self.gz) + "\t" + str(self.ax) + "\t" + str(self.ay) + "\t" + str(self.az) + "\t" + \
               str(self.mx) + "\t" + str(self.my) + "\t" + str(self.mz) + "\t"

    def to_json(self):
        return  str(
                    { "Limb": self.name,
		      "data": {"orientation": str(self.orientation),
                               "start": str(self.start),
                               "end": str(self.end)
                              }
                }
            )
            


def update_positions():

    def get_end_position(start, q, length):
        i, j, k = q.to_unit_vector()
        x_0 = i * length
        y_0 = j * length
        z_0 = k * length
        x, y, z = start
        x += x_0
        y += y_0
        z += z_0
        return x, y, z

    def get_mid_position(limb):
        x, y, z = limb.start 
        i, j, k = limb.end
        x = (i - x) / 2
        y = (j - y) / 2
        z = (k - z) / 2
        return x, y, z

    def get_orthogonal(start, q, length):
        v = q.to_unit_vector()
        x, y, z = v
        v = numpy.array([x, y, z])
        # make some vector not in the same direction as v
        not_v = numpy.array([1, 0, 0])
        if (v == not_v).all():
            not_v = numpy.array([0, 1, 0])

        # make vector perpendicular to v
        n1 = numpy.cross(v, not_v)
        # normalize n1
        if norm(n1) != 0:
            n1 /= norm(n1)
        #else:
            #print("Divide by zero error!")

        # make unit vector perpendicular to v and n1
        n2 = numpy.cross(v, n1)
        x, y, z = start
        x += n2[0] * length
        y += n2[1] * length
        z += n2[2] * length
        return x, y, z

    lower_torso.start = (0, 0, left_shin.length + left_thigh.length)
    upper_torso.start = get_end_position(lower_torso.start,
                                         upper_torso.orientation,
                                         upper_torso.length)
    tx, ty, tz = get_orthogonal(lower_torso.start, lower_torso.orientation,
                                hip_width)
    d = tz - lower_torso.start[2]
    left_thigh.start = (-tx, -ty, tz - d)
    left_shin.start = get_end_position(left_thigh.start, 
                                       left_thigh.orientation,
                                       left_thigh.length)
    left_foot.start = get_end_position(left_shin.start, 
                                       left_shin.orientation,
                                       left_shin.length)
    right_thigh.start = (tx, ty, tz)
    right_shin.start = get_end_position(right_thigh.start,
                                        right_thigh.orientation,
                                        right_thigh.length)
    right_foot.start = get_end_position(right_shin.start,
                                        right_shin.orientation,
                                        right_shin.length)
    head.start = get_end_position(upper_torso.start, 
                                  upper_torso.orientation,
                                  upper_torso.length)
    tx, ty, tz = get_orthogonal(head.start, 
                                upper_torso.orientation, 
                                shoulder_width)
    d = tz - head.start[2]
    left_bicep.start = -tx, -ty, tz - d
    left_forearm.start = get_end_position(left_bicep.start,
                                          left_bicep.orientation,
                                          left_bicep.length)
    left_hand.start = get_end_position(left_forearm.start,
                                       left_forearm.orientation,
                                       left_forearm.length)
    right_bicep.start = (tx, ty, tz)
    right_forearm.start = get_end_position(right_bicep.start,
                                              right_bicep.orientation,
                                              right_bicep.length)
    right_hand.start = get_end_position(right_forearm.start,
                                           right_forearm.orientation,
                                           right_forearm.length)
    # Save end positions.
    lower_torso.end = upper_torso.start
    upper_torso.end = head.start
    right_bicep.end = right_forearm.start
    right_forearm.end = right_hand.start
    right_hand.end = get_end_position(right_hand.start,
                                           right_hand.orientation,
                                           right_hand.length)
    right_thigh.end = right_shin.start
    right_shin.end = right_foot.start 
    right_foot.end = get_end_position(right_foot.start,
                                           right_foot.orientation,
                                           right_foot.length)
    left_bicep.end = left_forearm.start
    left_forearm.end = left_hand.start
    left_hand.end = get_end_position(left_hand.start,
                                           left_hand.orientation,
                                           left_hand.length)
    left_thigh.end = left_shin.start
    left_shin.end = left_foot.start 
    left_foot.end = get_end_position(left_foot.start,
                                           left_foot.orientation,
                                           left_foot.length)

    lower_torso.midpoint = get_mid_position(lower_torso)
    upper_torso.midpoint = get_mid_position(upper_torso)
    right_bicep.midpoint = get_mid_position(right_bicep)
    right_forearm.midpoint = get_mid_position(right_forearm)
    right_hand.midpoint = get_mid_position(right_hand)
    right_thigh.midpoint = get_mid_position(right_thigh)
    right_shin.midpoint = get_mid_position(right_shin)
    right_foot.midpoint = get_mid_position(right_foot)
    left_bicep.midpoint = get_mid_position(left_bicep)
    left_forearm.midpoint = get_mid_position(left_forearm)
    left_hand.midpoint = get_mid_position(left_hand)
    left_thigh.midpoint = get_mid_position(left_thigh)
    left_shin.midpoint = get_mid_position(left_shin)
    left_foot.midpoint = get_mid_position(left_foot)

    right_forearm.orthog = get_orthogonal(right_forearm.midpoint, right_forearm.orientation, 1)

    
def update_colliders():
    def update(limb, collider):
        collider.get_location(limb.midpoint, limb.orientation)
        return collider.intersect(limb.orthog, limb.midpoint, limb.length)

    timer_delay = 60
    result = update(right_forearm, right_forearm_collider)
    #print("result\n")    #vcot
    fileDump.write(str(result) + "\n")
    if ard:
        current = time.time()
        print("in ard\n")
        fileDump.write("in ard\n")
        if result:
            print("in result\n")
            fileDump.write("in result\n")
            if right_forearm_collider.last_time == None:
                right_forearm_collider.last_time = current
                print("event triggered")
                fileDump.write("event triggered\n")
                arduino.write(b'H')
            elif current - right_forearm_collider.last_time > timer_delay:
                right_forearm_collider.last_time = current
                print("event triggered")
                fileDump.write("event triggered\n")
                arduino.write(b'H')
    
def check_g_vars():
    global count, prev_t
    current = time.time()
    x, y, z = head.start
    height = z
    #print(height)   #vcot
    x, y, z = left_hand.end
    if z > height and (current - prev_t)>10:
        count += 1
        if count > 3:
            prev_t = current
            count = 0
            print("true\n")
            fileDump.write("true"+ "\n")
            if ard:
                print("event triggered")
                fileDump.write("event triggered\n")
                arduino.write(b'H')            
    else:
        #print("false\n")   #vcot
        count = 0
def sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz, weight = .85):
    x = z = 0

    # Polar Acceleration
    roll = math.atan2(az, ay)
    magnitude = math.sqrt(az ** 2 + ay ** 2)
    tilt = math.atan2(ax, magnitude)
    magnitude = math.sqrt(ax ** 2 + magnitude ** 2) * .00390625

    # Acceleration Quaternion
    w = math.cos(roll * .5)
    y = math.sin(roll * .5)
    qri = Quaternion.conjugate(Quaternion(w, x, y, z))
    w = math.cos(tilt * .5)
    y = math.sin(tilt * .5)
    qti = Quaternion.conjugate(Quaternion(w, x, y, z))
    qa = Quaternion.hamilton_product(qti, qri)

    # Polar Magnometer
    roll = math.atan2(mz, my) - math.pi
    magnitude = math.sqrt(mz ** 2 + my ** 2)
    tilt = math.atan2(mx, magnitude)

    # Magnometer Quaternion
    w = math.cos(roll * .5)
    y = math.cos(roll * .5)
    qr = Quaternion(w, x, y, z)
    w = math.cos(tilt * .5)
    y = math.cos(tilt * .5)
    qt = Quaternion(w, x, y, z)
    qm = Quaternion.hamilton_product(qt, qr)

    # Rotate Quaternion
    qm = Quaternion.hamilton_product(qm, qri)
    qm = Quaternion.hamilton_product(qm, qti)

    # Azimuth
    w, x, y = qm.get_quadruple()[:3]
    azimuth = math.atan2(x, y) + math.pi
    if w > 0:
        azimuth += math.pi
    azimuth = math.fmod(azimuth, 2 * math.pi) - math.pi

    # Finalize Magnometer Quaternion
    w = math.cos(azimuth * .5)
    z = math.sin(azimuth * .5)
    x = y = 0
    qm = Quaternion(w, x, y, z)

    # Combine magnometer and acceletation quaternions
    qam = Quaternion.hamilton_product(qm, qa)

    # Gyro Quaternion
    w = math.cos((gx + gy + gz) * .5)
    x = math.sin(gz * .5)
    y = math.sin(gx * .5)
    z = math.sin(gy * .5)
    qg = Quaternion(w, x, y, z)
    return qg
    x, y, z = qg.to_unit_vector()
    #print("x: ", x," y: ", y," z: ", z)

    delay = prev = Quaternion.hamilton_product(prev, qg)
    prev = Quaternion.slerp(qam, prev, weight)
    return Quaternion.min_distance(delay, prev)


def sensor_handler(address, *args):
    gx, gy, gz, ax, ay, az, mx, my, mz = args

    global gaitDetectRight, gaitDetectLeft
    global prev_gait_stage, step_count, swing_total_count, current_swing_count
    # Left Bicep
    if (address == "/Chordata/raw/kc-CH_1-0"):
        left_bicep.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
	        s.send(bytes(left_bicep.to_json(), 'UTF-8'))
        #print("left bicep : ",left_bicep.to_string())
        #fileDump.write(("left bicep : "+left_bicep.to_string()))
    # Left Forearm
    elif (address == "/Chordata/raw/kc-CH_1-1"):
        left_forearm.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_forearm.to_json(), 'UTF-8'))
        #print("left forearm : ",left_forearm.to_string())
        #fileDump.write(("left forearm : "+left_forearm.to_string()))
    # Left Hand
    elif (address == "/Chordata/raw/kc-CH_1-2"):
        left_hand.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_hand.to_json(), 'UTF-8'))
        #print("left hand : ",left_hand.to_string())
        #fileDump.write(("left hand : "+left_hand.to_string()))
    # Head
    elif (address == "/Chordata/raw/kc-CH_2-1"):
        head.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(head.to_json(), 'UTF-8'))
        #print(head.to_string())
        #fileDump.write(head.to_string())
    # Upper Torso
    elif (address == "/Chordata/raw/kc-CH_2-2"):
        upper_torso.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(upper_torso.to_json(), 'UTF-8'))
        #print(upper_torso.ax)
        #fileDump.write(upper_torso.to_string())
    # Right Bicep
    elif (address == "/Chordata/raw/kc-CH_3-0"):
        right_bicep.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_bicep.to_json(), 'UTF-8'))
        #print(right_bicep.to_string())
        #fileDump.write(right_bicep.to_string())
    # Right Forearm
    elif (address == "/Chordata/raw/kc-CH_3-1"):
        right_forearm.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_forearm.to_json(), 'UTF-8'))
        #print(right_forearm.to_string())
        #fileDump.write(right_forearm.to_string())
    # Right Hand
    elif (address == "/Chordata/raw/kc-CH_3-2"):
        right_hand.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_hand.to_json(), 'UTF-8'))
        #print(right_hand.to_string())
        #fileDump.write(right_hand.to_string())
    # Left Thigh
    elif (address == "/Chordata/raw/kc-CH_4-0"):
        left_thigh.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_thigh.to_json(), 'UTF-8'))
        #print(left_thigh.to_string())
        #fileDump.write(left_thigh.to_string())
    # Left Shin
    elif (address == "/Chordata/raw/kc-CH_4-1"):
        left_shin.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_shin.to_json(), 'UTF-8'))
        #print(left_shin.to_string())
        #fileDump.write(left_shin.to_string())
    # Left Foot
    elif (address == "/Chordata/raw/kc-CH_4-2"):
        left_foot.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_foot.to_json(), 'UTF-8'))
        #print(left_foot.to_string())
        #fileDump.write(left_foot.to_string())
    # Lower Torso
    elif (address == "/Chordata/raw/kc-CH_5-0"):
        lower_torso.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(lower_torso.to_json(), 'UTF-8'))
        #print(lower_torso.to_string())
        #fileDump.write(lower_torso.to_string())
    # Right Thigh
    elif (address == "/Chordata/raw/kc-CH_6-0"):
        right_thigh.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_thigh.to_json(), 'UTF-8'))
        #print(right_thigh.to_string())
        #fileDump.write(right_thigh.to_string())
    # Right Shin
    elif (address == "/Chordata/raw/kc-CH_6-1"):
        right_shin.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_shin.to_json(), 'UTF-8'))
        #print(right_shin.to_string())
        #fileDump.write(right_shin.to_string())
    # Right Foot
    elif (address == "/Chordata/raw/kc-CH_6-2"):
        right_foot.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_foot.to_json(), 'UTF-8'))
        #print(right_foot.to_string())
        #fileDump.write(right_foot.to_string())
    else:
        print("Unknown address:", address)
        #fileDump.write("Unknown address: "+str(address))
    update_positions()
    if collide:
        update_colliders()
    if g_vars:
        check_g_vars()



    # VSV additions 26/2/2023
    
    #print(right_thigh.gz,end='\t')
    #print(right_shin.gz)

    timeCurrent = time.time()

    #gaitDetectRight = gaitDetect()
    #gaitDetectLeft = gaitDetect()
    #print(upper_torso.ax)
    gaitDetectRight.testVal(right_thigh.gz,right_shin.gz,right_foot.gz)    #(upper_torso.gz,upper_torso.gy,upper_torso.gx) is just for testing --- actual arguments would be (right_thigh.gz,right_shin.gz,right_foot.gz)
    #print(gaitDetectRight.gaitStage,end='\t')
    gaitDetectLeft.testVal(left_thigh.gz,left_shin.gz,left_foot.gz)    #(upper_torso.gz,upper_torso.gy,upper_torso.gx) is just for testing --- actual arguments would be (left_thigh.gz,left_shin.gz,left_foot.gz)
    #print(gaitDetectLeft.gaitStage,end='\t')
    #print(right_foot.gz) #,right_shin.gz,right_foot.gz,sep='\t',end='\t')
    #print()
    #print(left_thigh.gx,left_shin.gx,left_foot.gx,sep='\t')
    #print("Executing")
    #print(trip_done_flag)
    
   
    
    global trip_done_flag
    ###------this is just test code
    #if(right_thigh.ay != None):
    #    if(right_thigh.ay<-4000 and trip_done_flag==0):
    #        client.publish('/leds/esp8266','TRIP'+str(trip_duration))
    #        print(right_thigh.ay)
    #        trip_done_flag=1
    #        print(trip_done_flag)

    #    else:
    #        print(right_thigh.ay)
    ###---------------------
    #print(right_thigh.ay)
    #print(gaitDetectRight.gaitStage)
   
    if leg_to_trip=="right":
        gaitDetectObj=gaitDetectRight
    elif leg_to_trip=="left":
        gaitDetectObj=gaitDetectLeft
    #print(gaitDetectObj.gaitStage)
    
    ### live detect code
    #dtw_out = detector_dtw.update(lower_torso.gz, timeCurrent, gaitDetectObj.gaitStage, output_type=dtw_output_type)
    #cwt_out = detector_cwt.update(lower_torso.gz, timeCurrent, output_type=cwt_output_type)
    
    if((gaitDetectObj.gaitStage-prev_gait_stage)==-2):   #updating step count at every heel strike
        step_count=step_count+1
        #print(step_count)
    prev_gait_stage = gaitDetectObj.gaitStage
    #print(step_count)
    ## trip_type and num_steps_to_trip come diretly from userinput.py (imported above)


    if((step_count==num_steps_to_trip-1)): # and (gaitDetectObj.gaitStage-prev_gait_stage)==1):    # checking first instant of toe off one step before trip trigger step is reahed 
        if(gaitDetectObj.gaitStage==2):
            swing_total_count = swing_total_count+1
            #print(swing_total_count)
    if((step_count==num_steps_to_trip)): # and (gaitDetectObj.gaitStage-prev_gait_stage)==1):  #checking if num_steps_to_trip is reached and swing has started
        #print("trip condition reached")
        if(gaitDetectObj.gaitStage==1):
            current_swing_count = current_swing_count+1
        if(trip_type==1 and current_swing_count>0.2*swing_total_count and trip_done_flag==0):    #trigger at 20 per cent of swing (trip type -- 1=early, 2=mid, 3=late trip)
            client.publish('/leds/esp8266',str(trip_duration))     #publishing through mqtt to esp to trigger trip client.publish(topic, message)
            trip_done_flag=1
            print(time.time())
        if(trip_type==2 and current_swing_count>0.45*swing_total_count and trip_done_flag==0):    #trigger at 45 per cent of swing
            client.publish('/leds/esp8266',str(trip_duration))     #publishing through mqtt to esp to trigger trip client.publish(topic, message)
            trip_done_flag=1
            print(time.time())
        if(trip_type==3 and current_swing_count>0.75*swing_total_count and trip_done_flag==0):    #trigger at 75 per cent of swing
            client.publish('/leds/esp8266',str(trip_duration))     #publishing through mqtt to esp to trigger trip client.publish(topic, message)
            trip_done_flag=1
            print(time.time())

    obj_list = [right_thigh,right_shin,right_foot,left_thigh,left_shin,left_foot,upper_torso,lower_torso]
    fileDump.write(str(np.round((timeCurrent-start_time),5))+"\t")

    for objj in obj_list:
        fileDump.write(objj.to_string())
    stage_data=str(gaitDetectRight.gaitStage)+"\t"+str(gaitDetectLeft.gaitStage)+"\t"
    fileDump.write(stage_data)
    fileDump.write(str(step_count)+"\n")
    
















def main():


    #mqtt_comm_func()
    LED_PIN=24
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.output(LED_PIN, GPIO.LOW)
    global client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.username_pw_set("trg", "chordata")
    client.connect('localhost',1883,60)
    client.loop_start()
    print('MQTT is running, press Ctrl+C to quit...')
    #client.publish('/leds/esp8266','TRIP'+str(trip_duration))

    #gait detect objects global
    global gaitDetectRight,gaitDetectLeft
    gaitDetectRight = gaitDetect()
    gaitDetectLeft = gaitDetect()

    # Limbs need to be accessed by sensor handler.
    global left_hand, left_forearm, left_bicep
    global right_hand, right_forearm, right_bicep
    global left_foot, left_shin, left_thigh
    global right_foot, right_shin, right_thigh
    global upper_torso, lower_torso, head, s
    global shoulder_width, hip_width
    global bluetooth, ard, collide, count, start_time
    global arduino, fileDump, g_vars, prev_t
    global step_count, swing_total_count, current_swing_count, prev_gait_stage, trip_done_flag
    global source_sig, detector_dtw, mother_wavelet, detector_cwt
    start_time=time.time()
    step_count=0
    swing_total_count=0
    current_swing_count=0
    prev_gait_stage = 0
    trip_done_flag=0
    #flag to set user input of limb lengths True= take user input | False= set default values
    ui_flag=False
    source_sig=SrcSignal()
    #source_sig.load_src(dtw_source_object_save)
    #detector_dtw = detect_dtw(source_sig, detect_thresh=dtw_loss_thresh, store_frames=dtw_store_frames)
    #detector_dtw.q1lim = q1lim
    #detector_dtw.q2lim = q2lim
    
    mother_vavelet=MotherWavelet()
    #mother_wavelet.load(cwt_source_obj_save)
    #detector_cwt = detect_cwt(mother_wavelet,scale=cwt_scale,scale_type="length",detect_thresh=cwt_detect_thresh,gy_store_frames=cwt_gy_store_frames,f_s=cwt_f_s,derivative=True,filter=True)

    

    fileDump = open("algDump.txt", "w+")
    fileDump.write("time"+"\t"+"r_thigh_gx"+"\t"+"r_thigh_gy"+"\t"+"r_thigh_gz"+"\t"+"r_thigh_ax"+"\t"+"r_thigh_ay"+"\t"+"r_thigh_az"+"\t"+"r_thigh_mx"+"\t"+"r_thigh_my"+"\t"+"r_thigh_mz"+"\t" \
                   +"r_shin_gx"+"\t"+"r_shin_gy"+"\t"+"r_shin_gz"+"\t"+"r_shin_ax"+"\t"+"r_shin_ay"+"\t"+"r_shin_az"+"\t"+"r_shin_mx"+"\t"+"r_shin_my"+"\t"+"r_shin_mz"+"\t" \
                   +"r_foot_gx"+"\t"+"r_foot_gy"+"\t"+"r_foot_gz"+"\t"+"r_foot_ax"+"\t"+"r_foot_ay"+"\t"+"r_foot_az"+"\t"+"r_foot_mx"+"\t"+"r_foot_my"+"\t"+"r_foot_mz"+"\t" \
                   +"l_thigh_gx"+"\t"+"l_thigh_gy"+"\t"+"l_thigh_gz"+"\t"+"l_thigh_ax"+"\t"+"l_thigh_ay"+"\t"+"l_thigh_az"+"\t"+"l_thigh_mx"+"\t"+"l_thigh_my"+"\t"+"l_thigh_mz"+"\t" \
                   +"l_shin_gx"+"\t"+"l_shin_gy"+"\t"+"l_shin_gz"+"\t"+"l_shin_ax"+"\t"+"l_shin_ay"+"\t"+"l_shin_az"+"\t"+"l_shin_mx"+"\t"+"l_shin_my"+"\t"+"l_shin_mz"+"\t" \
                   +"l_foot_gx"+"\t"+"l_foot_gy"+"\t"+"l_foot_gz"+"\t"+"l_foot_ax"+"\t"+"l_foot_ay"+"\t"+"l_foot_az"+"\t"+"l_foot_mx"+"\t"+"l_foot_my"+"\t"+"l_foot_mz"+"\t"\
                   +"up_torso_gx"+"\t"+"up_torso_gy"+"\t"+"up_torso_gz"+"\t"+"up_torso_ax"+"\t"+"up_torso_ay"+"\t"+"up_torso_az"+"\t"+"up_torso_mx"+"\t"+"up_torso_my"+"\t"+"up_torso_mz"+\
                   "\t" \
                   +"lo_torso_gx"+"\t"+"lo_torso_gy"+"\t"+"lo_torso_gz"+"\t"+"lo_torso_ax"+"\t"+"lo_torso_ay"+"\t"+"lo_torso_az"+"\t"+"lo_torso_mx"+"\t"+"lo_torso_my"+"\t"\
                   +"lo_torso_mz"+"\t" \
                   +"gaitStageRight"+"\t"+"gaitStageLeft"+"\t"+"step_count"+"\n")
    prev_t = time.time()
    count = 0
    #Mode Toggle
    bluetooth = False
    ard = False
    collide = False
    g_vars = True

    if ard:
        arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=.1)
        print("Arduino Started")
        fileDump.write("Arduino Started\n")
        arduino.write(b'H')

    left_hand = Limb("left_hand")
    left_forearm = Limb("left_forearm")
    left_bicep = Limb("left_bicep")
    right_hand = Limb("right_hand")
    right_forearm = Limb("right_forearm")
    right_bicep = Limb("right_bicep")
    left_foot = Limb("left_foot")
    left_shin = Limb("left_shin")
    left_thigh = Limb("left_thigh")
    right_foot = Limb("right_foot")
    right_shin = Limb("right_shin")
    right_thigh = Limb("right_thigh")
    upper_torso = Limb("upper_torso")
    lower_torso = Limb("lower_torso")
    head = Limb("head")






    # Set user length. Initialize Limbs.
    if ui_flag==True:
        d = input("Hand length: ")
        left_hand.length = int(d)
        right_hand.length = int(d)
        d = input("Forearm length: ")
        left_forearm.length = int(d)
        right_forearm.length = int(d)
        d = input("Bicep length: ")
        left_bicep.length = int(d)
        right_bicep.length = int(d)
        d = input("Torso length: ")
        upper_torso.length = int(d) / 2
        lower_torso.length = int(d) / 2
        d = input("Thigh length: ")
        left_thigh.length = int(d)
        right_thigh.length = int(d)
        d = input("Shin length: ")
        left_shin.length = int(d)
        right_shin.length = int(d)
        d = input("Foot length: ")
        left_foot.length = int(d)
        right_foot.length = int(d)
        d = input("Head length: ")
        head.length = int(d)
        # Set user width.
        shoulder_width = int(input("Shoulder width: "))
        hip_width = int(input("Hip width: "))
    else:

        left_hand.length = 1
        right_hand.length = 1

        left_forearm.length = 1
        right_forearm.length = 1

        left_bicep.length = 1
        right_bicep.length = 1

        upper_torso.length = 1
        lower_torso.length = 1

        left_thigh.length = 1
        right_thigh.length = 1

        left_shin.length = 1
        right_shin.length = 1

        left_foot.length = 1
        right_foot.length = 1

        head.length = 1

        shoulder_width=1
        hip_width=1



    # Calibrate
    input("Stand in a T-pose. Press enter to calibrate.")

    #print(f"U:\t{upper_torso.az}\t{upper_torso.ay}\t{upper_torso.ax}")


    # Set initial orientation.
    left_hand.orientation = Quaternion(0, 0, .7071, .7071)
    left_forearm.orientation = Quaternion(0, 0, .7071, .7071)
    left_bicep.orientation = Quaternion(0, 0, .7071, .7071)
    left_thigh.orientation = Quaternion(0, .7071, .7071, 0)
    left_shin.orientation = Quaternion(0, .7071, .7071, 0)
    left_foot.orientation = Quaternion(0, .7071, 0, .7071)
    right_hand.orientation = Quaternion(0, 0, .7071, -.7071)
    right_forearm.orientation = Quaternion(0, 0, .7071, -.7071)
    right_bicep.orientation = Quaternion(0, 0, .7071, -.7071)
    right_thigh.orientation = Quaternion(0, .7071, .7071, 0)
    right_shin.orientation = Quaternion(0, .7071, .7071, 0)
    right_foot.orientation = Quaternion(0, .7071, 0, .7071)
    head.orientation = Quaternion()
    upper_torso.orientation = Quaternion()
    lower_torso.orientation = Quaternion()

    update_positions()



    # test print raw acceleration value
    #print(upper_torso.ax)


    if collide:
        global left_hand_collider, left_forearm_collider, left_bicep_collider
        global right_hand_collider, right_forearm_collider, right_bicep_collider
        global left_foot_collider, left_shin_collider, left_thigh_collider
        global right_foot_collider, right_shin_collider, right_thigh_collider
        global upper_torso_collider, lower_torso_collider, head_collider

        global left_hand_collider_enable, left_forearm_collider_enable, left_bicep_collider_enable
        global right_hand_collider_enable, right_forearm_collider_enable, right_bicep_collider_enable
        global left_foot_collider_enable, left_shin_collider_enable, left_thigh_collider_enable
        global right_foot_collider_enable, right_shin_collider_enable, right_thigh_collider_enable
        global upper_torso_collider_enable, lower_torso_collider_enable, head_collider_enable

        left_hand_collider_enable = False
        left_forearm_collider_enable = False
        left_bicep_collider_enable = False
        right_hand_collider_enable = False
        right_forearm_collider_enable = False
        right_bicep_collider_enable = False
        left_foot_collider_enable = False
        left_shin_collider_enable = False
        left_thigh_collider_enable = False
        right_foot_collider_enable = False
        right_shin_collider_enable = False
        right_thigh_collider_enable = False
        upper_torso_collider_enable = False
        lower_torso_collider_enable = False
        head_collider_enable = False

        right_forearm_collider = Collider(right_forearm.length, .25, 
                 1, 0, 1, 0, right_forearm.orientation, right_forearm.midpoint)


    if bluetooth:
        serverMACAddress = 'A4:B1:C1:33:DB:27' # Desktop
        #serverMACAddress = '30:24:32:7D:36:46' # Laptop
        port = 4
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.connect((serverMACAddress,port))

    #checks if trip was executed once 
    trip_done_flag=0

    dispatcher = Dispatcher()
    dispatcher.map("/Chordata/raw/*", sensor_handler)
    ip = "127.0.0.1"
    port = 6565
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()



if __name__ == "__main__":
    main()

