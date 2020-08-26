#!/usr/bin/env python3

#Importing Custom Functions
from gaitDetectClass import * #class gaitDetect, func testVal(shank gyZ, heel gyZ)
from packageHandlerFunc import * #package_handler_raw(tup)
from sensorClass import * #class sensorObject, newValues(valueArray), angularAccCalc(), angleCalc(gaitDetectObject)
from serialSend import * #ardno(msg as string)
from slipAlgorithmFunc import * #slipAlgorithm(pelvis_forward_acc, heel_forward_acc, L_hh)

#Importing python libraries
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
import serial
import time
from math import sin, cos, sqrt, atan2
import numpy as np

#These will all be passed, rather than global eventually.
global timeCurrent, varType, timeStart
global dataDict, flagDict, toggleFlagDict
global packetReady, rPacketReady, passToAlgorithm, packetWasReady
global fileDump
global objRHeel, objRShank, objRThigh
global objLHeel, objLShank, objLThigh
global gaitDetectRight, gaitDetectLeft
global objects
global hip_heel_length
global intelNUCserial

#-----------------------------------#
#USER INPUTS
ip = "localhost"
port = 6565

intelNUCport = ''
intelNUCbaud = 115200

hip_heel_length = 1 #meters

#Currently unused
mass = 60 #kg
height = 180 #cm
#-----------------------------------#

#Turns data collection for particular sensors on/off if necessary.
toggleFlagDict = {
    "rThigh": True,
    "rShank": True,
    "rHeel": True,
    "lThigh": True,
    "lShank": True,
    "lHeel": True,
    "lowBack": True,
}


#Variable Initializations

packetReady = False
rPacketReady = False
packetWasReady = True
timeCurrent = time.time()
timeStart = time.time()


dataDict = {
    "rThigh":  [],
    "rShank":  [],
    "rHeel":  [],
    "lThigh": [],
    "lShank": [],
    "lHeel": [],
    "lowBack":  [],
}


flagDict = {
    "rThigh": False,
    "rShank": False,
    "rHeel": False,
    "lThigh": False,
    "lShank": False,
    "lHeel": False,
    "lowBack": False,
}


addressDict = {
    "10": "rThigh",
    "11": "rShank",
    "12": "rHeel",
    "30": "lThigh",
    "31": "lShank",
    "32": "lHeel",
    "20": "lowBack",
}


orderDict = {
    0: "rThigh",
    1: "rShank",
    2: "rHeel",
    3: "lThigh",
    4: "lShank",
    5: "lHeel",
    6: "lowBack",
}


passToAlgorithm = {
    "rt_raw": [],
    "rs_raw": [],
    "rh_raw": [],
    "lt_raw": [],
    "ls_raw": [],
    "lh_raw": [],
    "b_raw": [],
}


#Function to handle OSC input
def data_handler(address, *args):
    global timeCurrent, varType, timeStart
    global dataDict, flagDict, toggleFlagDict
    global packetReady, rPacketReady, passToAlgorithm, packetWasReady
    global fileDump
    global objRHeel, objRShank, objRThigh
    global objLHeel, objLShank, objLThigh
    global gaitDetectRight, gaitDetectLeft
    global objects
    global hip_heel_length
    global intelNUCserial
    
    out = []

#SERIAL SEND END OF PACKET------------------------------------------------------------------------------------------------
#If packet just finished sending or if first run, send start character (STX) over serial first.
#    if packetWasReady:
#        packetWasReady = False
#        send_over_serial([f"/x02"], intelNUCserial)
#--------------------------------------------------------------------------------------------------------------------------
	
#Collects variable type and sensor address as numbers
    varType = address[10]
    addr = ''
    addr += str(address[len(address) - 3])
    addr += str(address[len(address) - 1])
    
    #Takes in individual data and assembles into easily indexable dictionary packages.
    if addr in addressDict:
        limb = addressDict[addr]

        if varType == "r":
            dataDict[limb] = package_handler_raw(args)
            flagDict[limb] = True

#SERIAL SEND------------------------------------------------
#Whenever new data is brought in, it cuts the magnetometer (change 0:6 to 0:9 if you want magnetometer included), inserts the 2-letter sensor code at the beginning, and sends it with function to intelNUCserial.
#To skip, comment send_over_serial()
#            serialArr = dataDict[limb]
#            serialArr = serialArr[0:6]
#            serialArr.insert(0,limb[0:2])
#            send_over_serial(serialArr, intelNUCserial)
#-----------------------------------------------------------

#Tests if all sensors have been received before assembling packet and sending to algorithm
        if flagDict == toggleFlagDict:                    
            for x in flagDict:
                flagDict[x] = False
            
            rPacketReady = True
    
    
#Assembles full packet so that entire IMU state can be analyzed by algorithm at once.
    if rPacketReady:
        rPacketReady = False
        
        passToAlgorithm["rt_raw"] = dataDict["rThigh"]
        passToAlgorithm["rs_raw"] = dataDict["rShank"]
        passToAlgorithm["rh_raw"] = dataDict["rHeel"]
        passToAlgorithm["lt_raw"] = dataDict["lThigh"]
        passToAlgorithm["ls_raw"] = dataDict["lShank"]
        passToAlgorithm["lh_raw"] = dataDict["lHeel"]
        passToAlgorithm["b_raw"]  = dataDict["lowBack"]
        
        packetReady = True
        
#----------------------------------------------------------------------------------------------------------------#    
#Code is broken into reader above and algorithm below for increased customization and ease of changing algorithm. Everything below this line is almost entirely customizable.
#If sensor orientations change, they can be changed in the code below.
        
#When complete system state is ready, run calculations
    if packetReady:
        packetWasReady = True
        packetReady = False
        timeLastRun = timeCurrent
        timeCurrent = time.time()
        timeToRun = timeCurrent - timeLastRun
        
#Update data
        rt_raw = passToAlgorithm['rt_raw']
        rs_raw = passToAlgorithm['rs_raw']
        rh_raw = passToAlgorithm['rh_raw']
        lt_raw = passToAlgorithm['lt_raw']
        ls_raw = passToAlgorithm['ls_raw']
        lh_raw = passToAlgorithm['lh_raw']
        b_raw  = passToAlgorithm['b_raw']
        
#Values are moved around to equalize the axes of the IMUs.
#Does not put IMUs on a global coordinate system.
#Only sets local axes to be the same for ease on analysis.
		
#Right Thigh - no flipped values
        objRThigh.gyX = rt_raw[0]
        objRThigh.gyY = rt_raw[1]
        objRThigh.gyZ = rt_raw[2]
                  
        objRThigh.acX = rt_raw[3]
        objRThigh.acY = rt_raw[4]
        objRThigh.acZ = rt_raw[5]
                  
        objRThigh.mgX = rt_raw[6]
        objRThigh.mgY = rt_raw[7]
        objRThigh.mgZ = rt_raw[8]
        
#Right Shank - no flipped values
        objRShank.gyX = rs_raw[0]
        objRShank.gyY = rs_raw[1]
        objRShank.gyZ = rs_raw[2]
                  
        objRShank.acX = rs_raw[3]
        objRShank.acY = rs_raw[4]
        objRShank.acZ = rs_raw[5]
                  
        objRShank.mgX = rs_raw[6]
        objRShank.mgY = rs_raw[7]
        objRShank.mgZ = rs_raw[8]
        
#Right Heel - X and Y axes flipped, X negated
        objRHeel.gyX = -rh_raw[1]
        objRHeel.gyY = rh_raw[0]
        objRHeel.gyZ = rh_raw[2]
        
        objRHeel.acX = -rh_raw[4]
        objRHeel.acY = rh_raw[3]
        objRHeel.acZ = rh_raw[5]
        
        objRHeel.mgX = -rh_raw[7]
        objRHeel.mgY = rh_raw[6]
        objRHeel.mgZ = rh_raw[8]
		
#Left Thigh - X and Z values negated
        objLThigh.gyX = -lt_raw[0]
        objLThigh.gyY =  lt_raw[1]
        objLThigh.gyZ = -lt_raw[2]
                               
        objLThigh.acX = -lt_raw[3]
        objLThigh.acY =  lt_raw[4]
        objLThigh.acZ = -lt_raw[5]
                               
        objLThigh.mgX = -lt_raw[6]
        objLThigh.mgY =  lt_raw[7]
        objLThigh.mgZ = -lt_raw[8]
        
#Left Shank - X and Z values negated		
        objLShank.gyX = -ls_raw[0]
        objLShank.gyY =  ls_raw[1]
        objLShank.gyZ = -ls_raw[2]
                               
        objLShank.acX = -ls_raw[3]
        objLShank.acY =  ls_raw[4]
        objLShank.acZ = -ls_raw[5]
                               
        objLShank.mgX = -ls_raw[6]
        objLShank.mgY =  ls_raw[7]
        objLShank.mgZ = -ls_raw[8]
                  
#Left Heel - X and Y axes flipped, then X, Y, and Z values negated.
        objLHeel.gyX = -lh_raw[1]
        objLHeel.gyY = -lh_raw[0]
        objLHeel.gyZ = -lh_raw[2]
                              
        objLHeel.acX = -lh_raw[4]
        objLHeel.acY = -lh_raw[3]
        objLHeel.acZ = -lh_raw[5]
                              
        objLHeel.mgX = -lh_raw[7]
        objLHeel.mgY = -lh_raw[6]
        objLHeel.mgZ = -lh_raw[8]
        
#Lower Back - X and Z values flipped, X value negated
        objLowBack.gyX = -b_raw[2]
        objLowBack.gyY =  b_raw[1]
        objLowBack.gyZ =  b_raw[0]
                       
        objLowBack.acX = -b_raw[5]
        objLowBack.acY =  b_raw[4]
        objLowBack.acZ =  b_raw[3]
                   
        objLowBack.mgX = -b_raw[8]
        objLowBack.mgY =  b_raw[7]
        objLowBack.mgZ =  b_raw[6]
        
        
#Run actual calculations contained in the objects
#Right and Left Gait Detection
        gaitDetectRight.testVal(objRShank.gyZ, objRHeel.gyZ)
        gaitDetectLeft.testVal(objLShank.gyZ, objLHeel.gyZ)

#Right Leg Angle Approximations
        objRThigh.angleCalc(gaitDetectRight)
        objRShank.angleCalc(gaitDetectRight)
        objRHeel.angleCalc(gaitDetectRight)

#Left Leg Angle Approximations
        objLThigh.angleCalc(gaitDetectLeft)
        objLShank.angleCalc(gaitDetectLeft)
        objLHeel.angleCalc(gaitDetectLeft)
		
#Append beginning of output string - time, time between measurements, right gait stage, left gait stage
        outputString = f"{time.time() - timeStart}\t{timeToRun}\t{gaitDetectRight.gaitStage}\t{gaitDetectLeft.gaitStage}\t\t"

#Cycle through all sensor objects to append formatted version of every sensor's raw data to output string
        for x in objects:
            outputString += f"{x.gyX}\t"
            outputString += f"{x.gyY}\t"
            outputString += f"{x.gyZ}\t\t"

            outputString += f"{x.acX}\t"
            outputString += f"{x.acY}\t"
            outputString += f"{x.acZ}\t\t"

            #outputString += f"{x.mgX}\t"
            #outputString += f"{x.mgY}\t"
            #outputString += f"{x.mgZ}\t\t"
			
			outputString += f"\t"

#Cycle through all sensor objects to append formatted version of every sensor's angle approximation to output string (note, back angle is not calculated and stays at 0)
        for x in objects:
            outputString += f"{x.zAngle}\t"

        outputString += f"\t"
        
#Calculates Slip Indicator from Trkov IFAC 2017 paper
        slipRight = gaitDetectRight.slipTrkov(objLowBack.acX, ((objRHeel.acX * np.cos(objRHeel.zAngle * .01745)) - (objRHeel.acY * np.sin(objRHeel.zAngle * .01745))), hip_heel_length)
        slipLeft = gaitDetectLeft.slipTrkov(objLowBack.acX, ((objLHeel.acX * np.cos(objLHeel.zAngle * .01745)) - (objLHeel.acY * np.sin(objLHeel.zAngle * .01745))), hip_heel_length)
		
#Appends final values to output string, print, and save to file.
        outputString += f"{slipRight}\t{slipLeft}\t\t"
	
        #for x in objects:
        #    outputString += f"{x.gravAngleSmoothed}\t"
        #    outputString += f"{x.angleFromGravity}\t\t"

        legForward, kneeAngleR, kneeAngleL = gaitDetectRight.kneelingDetection(objRThigh, objRShank, objRHeel, objLThigh, objLShank, objLHeel)
        outputString += f"{kneeAngleR}\t{kneeAngleL}\t\t{legForward}"
        outputString += f"\n"
		
        print(outputString)
        fileDump.write(f"{outputString}")
		

#SERIAL SEND--------------------------------------------
#Sends all processed data over serial.
#comment send_over_serial to skip
#        serialArr = ["PR", time.time() - timeStart, timeToRun, gaitDetectRight.gaitStage, gaitDetectLeft.gaitStage, slipRight / (10**26), slipLeft / (10**26)]
#        send_over_serial(serialArr, intelNUCserial)
#---------------------------------------------------


#Handles any OSC messages that aren't picked up by dataHandler
def default_handler(address, *args):
    out = 'DEFAULT '
    out += str(address)
    out += str(args)


#Sets up OSC server
def main_func(ip, port):   
    dispatcher = Dispatcher()
    dispatcher.map("/Chordata/r*", data_handler)
    dispatcher.set_default_handler(default_handler)
    
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()  # Blocks forever



if __name__ == "__main__":    
    
    #Variable initializations
    #serial object for NUC. Comment out if not used.
    #intelNUCserial = serial.Serial(intelNUCport, intelNUCbaud)
	
    #create objects for sensor operations and value storage.
    objRThigh = sensorObject()
    objRShank = sensorObject()
    objRHeel = sensorObject()
	
    objLThigh = sensorObject()
    objLShank = sensorObject()
    objLHeel = sensorObject()
	
    objLowBack = sensorObject()
    
    #create gait detect objects for each leg
    gaitDetectRight = gaitDetect()
    gaitDetectLeft = gaitDetect()
	
    #create lists that can be cycles through to iterate over every object, as well as create the file data header.
    objects = [objRThigh, objRShank, objRHeel, objLThigh, objLShank, objLHeel, objLowBack]
    stringObjects = ["RThigh", "RShank", "RHeel", "LThigh", "LShank", "LHeel", "LowBack"]
    stringAxes = ["x","y","z"]
    #stringSensors = ["gy","ac","mg"]
    stringSensors = ["gy","ac"]
    #Create formatted file header
    fileDump = open("algDump.txt", "w+")
    header = "time\ttimeToRun\tgaitStageR\tgaitStageL\t\t"
    for x in stringObjects:
        for y in stringSensors:
            for z in stringAxes:
                header += f"{y}/{z}/{x}\t"
            header += f"\t"
        header += f"\t"
	
    for x in stringObjects:
        header += f"zAngle{x}\t"
		
    header += f"\tslipRight\tslipLeft\t"
    header += f"\tKneeAngleR\tKneeAngleL\t"
    header += f"\tKneelingIndicator\t"
	
    header += f"\n"
    fileDump.write(header)
    
    main_func(ip, port)
    client.close()
