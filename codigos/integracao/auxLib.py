# -*- coding: utf-8 -*-
"""
Auxiliar library for the Serial and User interface for the Helmholtz 3Dcoil project

Tarcis Becher
CTR
"""

#%% Imports ---------------------------------------------------------------------------------------------------------

import os
import numpy as np
import re
import datetime
import visa
import serial
import serial.tools.list_ports
from time import sleep

# -------------------------------------------------------------------------------------------------------------------
#%% Definitions -----------------------------------------------------------------------------------------------------

MCUcomText = 'STMicroelectronics STLink Virtual COM Port'
csvFileDir = "./csvData/"
regexSampleFinder = "(-?\w*.\w*);"

# -------------------------------------------------------------------------------------------------------------------
#%% Auxiliar function -----------------------------------------------------------------------------------------------
class CurrentCalculatorObj():
      
    def __init__(self, channel, zeroField, uT_to_mA, mA_to_uT, infCurrentLim = 0.1, supCurrentLim = 500):
        
        self.channel = channel
        self.infCurrentLim = infCurrentLim # mA
        self.supCurrentLim = supCurrentLim # mA
        self.zeroField = zeroField
        self.uT_to_mA = uT_to_mA
        self.mA_to_uT = mA_to_uT
        
        if self.zeroField > 0:
            self.zeroFieldSense = 1
        else:
            self.zeroFieldSense = -1
            
        self.incrementSense = 1

    
    def calculateCurrent(self, Field):
        if Field >= self.zeroField:
            self.incrementSense = 1
        else :
            self.incrementSense = -1
        
        current = (self.zeroField*self.zeroFieldSense + Field*self.incrementSense)*self.uT_to_mA
        
        if self.zeroField > 0:
            current = -current
            
        #print(current)
        
        return current
       
    
    # sets'n gets
    def set_uT_to_mA(self, new_uT_to_mA):
        self.uT_to_mA = new_uT_to_mA
        
    def get_uT_to_mA(self):
        return self.uT_to_mA
    
    def set_mA_to_uT(self, new_mA_to_uT):
        self.mA_to_uT = new_mA_to_uT
        
    def get_mA_to_uT(self):
        return self.mA_to_uT
    
    def setSupCurrentLim(self, newsupCurrentLim):
        self.supCurrentLim = newsupCurrentLim
        
    def getSupCurrentLim(self):
        return self.supCurrentLim
    
    def setInfCurrentLim(self, newinfCurrentLim):
        self.infCurrentLim = newinfCurrentLim
        
    def getInfCurrentLim(self):
        return self.infCurrentLim
    
    def setZeroFieldSense(self, newzeroFieldSense):
        self.zeroFieldSense = newzeroFieldSense
        
    def getZeroFieldSense(self):
        return self.zeroFieldSense
    
    def setSentidoDeIncremento(self, newSentidoDeIncremento):
        self.incrementSense = newSentidoDeIncremento
        
    def getSentidoDeIncremento(self):
        return self.incrementSense

    def setZeroField(self, newZeroField):
        self.zeroField = newZeroField
        
    def getZeroField(self):
        return self.zeroField
    
    def setChannel(self, newChannel):
        self.channel = newChannel
        
    def getChannel(self):
        return self.channel
    
    
class storeDataObj:
    def __init__(self, rotationMatrix):
        self.current = list()
        self.field = [list(),list(),list()]
        self.channel = list()
        self.rotationMatrix = rotationMatrix
        self.zeroField = list()
        self.control = list()
        self.index = 0
        
    def storeData(self, field, current, channel, zeroField, control):
        self.field[0].append(field[0])
        self.field[1].append(field[1])
        self.field[2].append(field[2])
        self.current.append(current)
        self.channel.append(channel)
        self.zeroField.append(zeroField)
        self.control.append(control)
        self.index += 1
    
    def createCSVfile(self, fileDir, fileName):
        #date = WhatTimeIsIt()
        #fileName = date[0:10] + "-" + date[11:13] + "-" + date[14:16] + "-" + date[17:19]
        #fileName = 'tests'
        makeDir(fileDir)
        fileCsv = createAndOpenFile(fileDir, fileName, ".csv")
        fileCsv.write("RotationMatrix;" + 
                      str(self.rotationMatrix[0][0]) + ";" + 
                      str(self.rotationMatrix[0][1]) + ";" + 
                      str(self.rotationMatrix[0][2]) + ";" + 
                      str(self.rotationMatrix[1][0]) + ";" + 
                      str(self.rotationMatrix[1][1]) + ";" + 
                      str(self.rotationMatrix[1][2]) + ";" + 
                      str(self.rotationMatrix[2][0]) + ";" + 
                      str(self.rotationMatrix[2][1]) + ";" + 
                      str(self.rotationMatrix[2][2]) + ";" + "\n")
        fileCsv.write("Axis X; Axis Y; Axis Z; Current; Channel; Date \n")
        i = 0
        #print('what')
        while (i <  len(self.current)):
            self.index -= 1
            data = [str(float(self.field[0][i])), str(float(self.field[1][i])), str(float(self.field[2][i])), 
                    str(float(self.current[i])), str(float(self.channel[i])), str(float(self.zeroField[i])), self.control[i]]
            #print(i)
            i += 1
            try:
                j = 0
                while (j < len(data)):
                    fileCsv.write(data[j])
                    fileCsv.write(";")
                    j+=1     
                fileCsv.write(WhatTimeIsIt())
                fileCsv.write("\n")
            except Exception as error:
                print(error)
            
        closeFile(fileCsv)
   
def closedLoopAdjust(visaObj, serialObj, channel, samplingTime, desiredField, rotationMatrix, initialValue, supLim, control = 'Current', breakCondition = 0.5, maxIterations = 20):
    
    iAnt = initialValue
    eAnt = 0
    index = 0
    
    
    while (1):
        
        i = eAnt*samplingTime + iAnt        
        
        if i > supLim:
            i = supLim
        if i < -supLim:
            i = -supLim
            
        if i >= 0:
            if channel == 1:
                changeCoilPolarity(serialObj, 'resetExternal')
            elif channel == 2:
                changeCoilPolarity(serialObj, 'resetInner')
            elif channel == 3:
                changeCoilPolarity(serialObj, 'resetMiddle')
        else:
            if channel == 1:
                changeCoilPolarity(serialObj, 'setExternal')
            elif channel == 2:
                changeCoilPolarity(serialObj, 'setInner')
            elif channel == 3:
                changeCoilPolarity(serialObj, 'setMiddle')
        if control == 'Current':
            setPowerSupplyChCurrent(visaObj, channel, float(i/1000)) 
        else:
            setPowerSupplyChVoltage(visaObj, channel, float(i/1000)) 
        
        #print('vish')
        fData = getMeasurement(serialObj)
        newField = np.matmul(rotationMatrix, fData)
        #print(newField)
        e = desiredField - newField[channel-1]
        #print(e)
        eAnt = e
        iAnt = i
        
        index += 1
        if (np.linalg.norm(e) < np.linalg.norm(breakCondition)):
            break
        if index > maxIterations:
            break    
        
    return newField, i, e



def setField(visaObj, serialObj, CurrentCalculatorObjObj, samplingTime, desiredField, rotationMatrix, breakCondition = 0.3, maxIterations = 20):
        
    current = CurrentCalculatorObjObj.calculateCurrent(desiredField)
    
    #infLim = CurrentCalculatorObjObj.getInfCurrentLim()
    supLim = CurrentCalculatorObjObj.getSupCurrentLim()
    #errorMessage = print("Current transpassed the limit, it's ajusted to de limit")
        
    channel = CurrentCalculatorObjObj.getChannel()
    
    voltFlag = 0
    if current < 0.1:
        if current > 0:
            voltFlag = 1
    if current > -0.1:
        if current < 0:
            voltFlag = 1
    
    if voltFlag :
        
        control = 'Voltage'
        setPowerSupplyChCurrent(visaObj, channel, float(supLim/1000)) 
        setPowerSupplyChVoltage(visaObj, channel, 0)
        
    else:
        control = 'Current'    
        setPowerSupplyChCurrent(visaObj, channel, float(current/1000)) 
        setPowerSupplyChVoltage(visaObj, channel, 4)
    
    if current >= 0:
        if channel == 1:
            changeCoilPolarity(serialObj, 'resetExternal')
        elif channel == 2:
            changeCoilPolarity(serialObj, 'resetInner')
        elif channel == 3:
            changeCoilPolarity(serialObj, 'resetMiddle')
    else:
        if channel == 1:
            changeCoilPolarity(serialObj, 'setExternal')
        elif channel == 2:
            changeCoilPolarity(serialObj, 'setInner')
        elif channel == 3:
            changeCoilPolarity(serialObj, 'setMiddle')
        #current = -current
           
    if control == 'Current':
        # closed loop control code     
        initialValue = current
    else:
        initialValue = 0
    
    newField, i, er = closedLoopAdjust(visaObj, serialObj, channel, samplingTime, desiredField, rotationMatrix, initialValue, supLim, control = control, breakCondition = breakCondition, maxIterations = maxIterations)
    
    #print(er)
        
    return newField, i, control 


def updateCoilPolarity(serialObj, current, channel):
    
    if current >= 0:
        if channel == 1:
            changeCoilPolarity(serialObj, 'resetExternal')
        elif channel == 2:
            changeCoilPolarity(serialObj, 'resetInner')
        elif channel == 3:
            changeCoilPolarity(serialObj, 'resetMiddle')
    else:
        
        if channel == 1:
            changeCoilPolarity(serialObj, 'setExternal')
        elif channel == 2:
            changeCoilPolarity(serialObj, 'setInner')
        elif channel == 3:
            changeCoilPolarity(serialObj, 'setMiddle')
            

def getDataFromReader(reader, dataOffset, rotationMatrixFlag = True):
    index = 0
    zeroField = np.zeros((3,1))
    channel = list()
    floatData = [list(), list(), list(), list()]
    rotationMatrixMeasures = np.zeros((3,3))
    control = list()
    j = 0
    for row in reader:
        flag0 = 1
        flag1 = 1
        flag2 = 1
        
        if rotationMatrixFlag:
            if index == 0:
                fData = strToFloat(row[1:10])
                rotationMatrixMeasures[0][0] = fData[0]
                rotationMatrixMeasures[0][1] = fData[1]
                rotationMatrixMeasures[0][2] = fData[2]
                rotationMatrixMeasures[1][0] = fData[3]
                rotationMatrixMeasures[1][1] = fData[4]
                rotationMatrixMeasures[1][2] = fData[5]
                rotationMatrixMeasures[2][0] = fData[6]
                rotationMatrixMeasures[2][1] = fData[7]
                rotationMatrixMeasures[2][2] = fData[8]
        if index > dataOffset:
            fData = strToFloat(row[0:5])
            floatData[0].append(fData[0])
            floatData[1].append(fData[1])
            floatData[2].append(fData[2])
            floatData[3].append(fData[3])
            channel.append(int(fData[4]))
            control.append(row[6:7])
            if j < 3:
                if fData[4] == 3:
                    if (flag0):
                        zeroField[2] = strToFloat(row[5:6])
                        j += 1
                        flag0 = 0
                if fData[4] == 2:
                    if (flag1):
                        zeroField[1] = strToFloat(row[5:6])
                        j += 1
                        flag1 = 0
                if fData[4] == 1:
                    if (flag2):
                        zeroField[0] = strToFloat(row[5:6])
                        j += 1
                        flag2 = 0
        index += 1
    return floatData, channel, zeroField, rotationMatrixMeasures, control


def rotationMatrixCalculator(visaObj, serialObj, teorethicalValue):
    
    # Rotation matrix, to align the measurements with the Coils axis
    fData = getMeasurement(serialObj)    
    # print(fData)
    
    # buffer to do the rotation matrix measurements
    bufferData = np.zeros((4,3))
    
    # Assuming that the measurement is precise enough
    bufferData[0,0] = fData[0]
    bufferData[0,1] = fData[1]
    bufferData[0,2] = fData[2]
    
    # Setting the voltage for the power supply
    setPowerSupplyAllChVoltage(visaObj, 4)
    
    # Current used in the coils to make the relationship between current and field measured
    baseCurrent = 400 #mA
    
    # Code to get the measurement information for the rotation matrix
    for i in range(3,0,-1):
        
        setPowerSupplyChCurrent(visaObj, i, baseCurrent/1000)
        sleep(0.1)
        
        fData = getMeasurement(serialObj)
        
        setPowerSupplyChCurrent(visaObj, i, 0)
        sleep(0.1)    
        
        bufferData[i,0] = fData[0] - bufferData[0,0]
        bufferData[i,1] = fData[1] - bufferData[0,1]
        bufferData[i,2] = fData[2] - bufferData[0,2]
        
    # Doing a linear relationship between the Current set in the coil and the Field measured with this 
    ch1Field = np.linalg.norm(bufferData[1,:])
    ch1Current = baseCurrent
    
    ch2Field = np.linalg.norm(bufferData[2,:])
    ch2Current = baseCurrent
    
    ch3Field = np.linalg.norm(bufferData[3,:])
    ch3Current = baseCurrent
    
    
    CurrentToField = np.zeros((3,1))
    FieldToCurrent = np.zeros((3,1))
    
    # Linear coeficients to converto from uT to mA and vice-versa
    CurrentToField[0] = ch1Field/ch1Current
    FieldToCurrent[0] = 1/CurrentToField[0]
    
    CurrentToField[1] = ch2Field/ch2Current
    FieldToCurrent[1] = 1/CurrentToField[1]
    
    CurrentToField[2] = ch3Field/ch3Current
    FieldToCurrent[2] = 1/CurrentToField[2]
    
    relationshipError = np.zeros((3,1))
    
    relationshipError[0] = np.linalg.norm(CurrentToField[0] - teorethicalValue[0])/teorethicalValue[0]*100
    relationshipError[1] = np.linalg.norm(CurrentToField[1] - teorethicalValue[1])/teorethicalValue[1]*100
    relationshipError[2] = np.linalg.norm(CurrentToField[2] - teorethicalValue[2])/teorethicalValue[2]*100
    
    #print("field/current ration error (%) for each coil")
    #print(relationshipError)
    
    # Those are the equations used to find the coeficients for the rotation matrix
    '''
    Ix0 = Cx1*xM0 + Cx2*yM0 + Cx3*zM0
    Ix1 = Cx1*xM1 + Cx2*yM1 + Cx3*zM1
    Ix2 = Cx1*xM2 + Cx2*yM2 + Cx3*zM2
    
    Iy0 = Cy1*xM0 + Cy2*yM0 + Cy3*zM0
    Iy1 = Cy1*xM1 + Cy2*yM1 + Cy3*zM1
    Iy2 = Cy1*xM2 + Cy2*yM2 + Cy3*zM2
    
    Iz0 = Cz1*xM0 + Cz2*yM0 + Cz3*zM0
    Iz1 = Cz1*xM1 + Cz2*yM1 + Cz3*zM1
    Iz2 = Cz1*xM2 + Cz2*yM2 + Cz3*zM2
    '''
    
    # The code to arrange the equations to get the rotation coeficients
    I1 = np.array([baseCurrent,0,0])
    I2 = np.array([0,baseCurrent,0])
    I3 = np.array([0,0,baseCurrent])
    M = np.zeros((3,3))
    
    for i in range(1,4):
        for j in range(0,3):
            M[i-1,j] = bufferData[i,j]
    
    # Solving the 3x3 systems
    C1 = np.linalg.solve(M,I1)*ch1Field/ch1Current
    C2 = np.linalg.solve(M,I2)*ch2Field/ch2Current
    C3 = np.linalg.solve(M,I3)*ch3Field/ch3Current
    
    # The rotation matrix 'C' that transform the uT measurement raw to the coils direction measurement in uT too
    rotationMatrix = [C1, C2, C3]
    
    # Reseting the Power Supply
    setPowerSupplyAllChCurrent(visaObj, 0)
    setPowerSupplyAllChVoltage(visaObj, 4)
    
    sleep(0.1)
    
    return rotationMatrix, CurrentToField, FieldToCurrent, relationshipError


def COMdetect(COMdescription = MCUcomText, errorMessage = 'MCU COM not detected'):
    xablau = serial.tools.list_ports.comports()
    for i in range(0,len(xablau)):
        #print(xablau[i],[1])
        ue = xablau[i][1].find(COMdescription)
        if ue != -1:
            COM = xablau[i][0]
            break
        else:
            COM = errorMessage
    return COM

def waitForNewData(serial, nTries = 1, tryTime = 0.001):
    
    i = 0
    while 1:
        if serial.in_waiting > 5:
            return 0
        else:
            sleep(tryTime)
            i += 1
        if i > nTries:
            #print('dataWaiting tries overflow')
            return 1

def getDataFromSerial(serial, regexSampleFinder):
    strXYZSamples = re.findall(regexSampleFinder, str(serial.readline()))
    #print(strXYZSamples)
    serial.reset_input_buffer()
    return strXYZSamples

def createSerialObj(COM):
    return serial.Serial(COM, timeout=0.5, baudrate = 256000)

def getDataFromSerialToString(serial, regexSampleFinder, time_ms):
    
    try:
        waitForNewData(serial,tryTime = float(time_ms)/1000)
            
        return getDataFromSerial(serial, regexSampleFinder)      
        #sleep(float(time_ms)/1000
    
    except Exception as error:
        print(error)
        return 0

def strToFloat(strData):
    i = 0
    floatData = np.ones((len(strData),1), float, 'C')
    while (i < len(strData)):
        floatData[i] = float(strData[i])
        i += 1
    return floatData

def WhatTimeIsIt():
    justTime = str(datetime.datetime.now())
    return justTime[0:22]

def writeDataOnFile(file, data = [0]):
    
    try:
        i = 0
        while (i < len(data)):
            file.write(data[i])
            file.write(";")
            i+=1     
        file.write(WhatTimeIsIt())
        file.write("\n")
        return 0
    except Exception as error:
        print(error)
        return -1

def makeDir(director):
    
    try:
        os.makedirs(director)
    except FileExistsError:
        # directory already exists
        pass

def createAndOpenFile(folder, name, ext):
    
    filePath = folder + name + ext
    
    return open(filePath,"w+")
    
def closeSerialObj(serial):
    serial.close()
    
def closeFile(file):
    file.close()
    
def sendCommandPS(instrument , command):
    instrument.write(command)  
    sleep(0.01)

def startupCoilsPolarity(serialObj):
    i = 0;
    serialObj.reset_input_buffer()
    serialObj.reset_output_buffer()
    while 1:
        serialObj.write(b'S')
        #print(i)
        i+=1
        if 0 == waitForNewData(serialObj):
            check = str(serialObj.readline())
            #print(check)
            serialObj.reset_input_buffer()
            if check == "b'S Done\\n'":
                break
        else:
            if i > 10:
                serialObj.reset_input_buffer()
                break
'''            
def adjustCurrent(visaObj, serialObj, C, coil, initialCurrent, desiredField, polarity, breakCondition = 0.1, nIteration = 40):
    
    # Setting the initial values of B and Ba
    i = 0   
    newCurrent = initialCurrent
    adjustCheck = 1
    if coil == 'External':
        index = 0
    elif coil == 'Inner':
        index = 1
        adjustCheck = 3
    elif coil == 'Middle':
        index = 2
    else:
        raise Exception("There's no coil with this reference")
        
    # The iteration loop that will find the correct HelmoltzCoil to get the I desired with B desired together
    while (1):
        
        setPowerSupplyChCurrent(visaObj, index+1, float(newCurrent/1000))
                    
        # Code that change the current and get a measurement to get the field
        fData = getMeasurement(serialObj)
        field = np.matmul(C, fData)
        
        field[index] = float(field[index])
        
        print(field)
        # --------------------------------------------------------------------------------------------
        # Increment the coilIntDiameter for the next iteration
        i += 1
        
        if (np.linalg.norm(field[index] - desiredField) < breakCondition):
            break
        
        if (i > nIteration):
            break
        
        check = np.linalg.norm(desiredField - field[index])
        #print(check)
        if check < 0.5*adjustCheck:
            if polarity > 0:
                if (field[index] < desiredField):
                    newCurrent +=  0.1 #mA
                elif (field[index] > desiredField):
                    newCurrent -=  0.1 #mA
            else:
                if (field[index] < desiredField):
                    newCurrent -=  0.1 #mA
                elif (field[index] > desiredField):
                    newCurrent +=  0.1 #mA
        elif check < 3*adjustCheck:
            if polarity > 0:
                if (field[index] < desiredField):
                    newCurrent +=  1 #mA
                elif (field[index] > desiredField):
                    newCurrent -=  1 #mA
            else:
                if (field[index] < desiredField):
                    newCurrent -=  1 #mA
                elif (field[index] > desiredField):
                    newCurrent +=  1 #mA
        else:
            if polarity > 0:
                if (field[index] < desiredField):
                    newCurrent +=  10 #mA
                elif (field[index] > desiredField):
                    newCurrent -=  10 #mA
            else:
                if (field[index] < desiredField):
                    newCurrent -=  10 #mA
                elif (field[index] > desiredField):
                    newCurrent +=  10 #mA
        
    # -----------------------------------------------------------------------------------------------------

    return newCurrent
'''
def changeCoilPolarity(serialObj, Coil):
    i = 0;
    serialObj.reset_input_buffer()
    serialObj.reset_output_buffer()
    while 1:
        if Coil == 'setExternal':
            serialObj.write(b'E')
        elif Coil == 'resetExternal':
            serialObj.write(b'e')
        elif Coil == 'setMiddle':
            serialObj.write(b'M')
        elif Coil == 'resetMiddle':
            serialObj.write(b'm')
        elif Coil == 'setInner':
            serialObj.write(b'I')
        elif Coil == 'resetInner':
            serialObj.write(b'i')
        else:
            raise Exception("This Coil reference doesn't exist")
            
        i+=1
        if 0 == waitForNewData(serialObj):
            check = str(serialObj.readline())
            #print(check)
            serialObj.reset_input_buffer()
            if check == "b'E Done\\n'":
                break
            if check == "b'I Done\\n'":
                break
            if check == "b'M Done\\n'":
                break
            if check == "b'e Done\\n'":
                break
            if check == "b'i Done\\n'":
                break
            if check == "b'm Done\\n'":
                break
        else:
            if i > 200:
                serialObj.reset_input_buffer()
                break
            
def inChangeCoilPolarity(serialObj):
    i = 0;
    serialObj.reset_input_buffer()
    serialObj.reset_output_buffer()
    while 1:
        serialObj.write(b'I')
        i+=1
        if 0 == waitForNewData(serialObj):
            serialObj.reset_input_buffer()
            break
        else:
            if i > 10:
                serialObj.reset_input_buffer()
                break

def midChangeCoilPolarity(serialObj):
    i = 0;
    serialObj.reset_input_buffer()
    serialObj.reset_output_buffer()
    while 1:
        serialObj.write(b'M')
        i+=1
        if 0 == waitForNewData(serialObj):
            serialObj.reset_input_buffer()
            break
        else:
            if i > 10:
                serialObj.reset_input_buffer()
                break
    
def exChangeCoilPolarity(serialObj):   
    i = 0;
    serialObj.reset_input_buffer()
    serialObj.reset_output_buffer()
    while 1:
        serialObj.write(b'E')
        i+=1
        if 0 == waitForNewData(serialObj):
            serialObj.reset_input_buffer()
            break
        else:
            if i > 10:
                serialObj.reset_input_buffer()
                break

def getMeasurement(serialObj):
    i = 0;
    serialObj.reset_input_buffer()
    while 1:
        fData = np.zeros((3,1))
        serialObj.write(b'D')
        i+=1
        if 0 == waitForNewData(serialObj):   
            strData = getDataFromSerial(serialObj, regexSampleFinder)
            if i == 8:
                fData[0] = 0
                fData[1] = 0
                fData[2] = 0
                return fData
            if len(strData) == 3:
                fData = strToFloat(strData)
            else:  
                print(strData)
                serialObj.reset_input_buffer()
                if i == 5:
                    fData[0] = 0
                    fData[1] = 0
                    fData[2] = 0
                    return fData
                continue
            serialObj.reset_input_buffer()
            return fData

def startUpPowerSupply(resource_name = 'USB0::0x1AB1::0x0E11::DP8A203800261::0::INSTR'):
    
    rm = visa.ResourceManager() # To connect the wraper to the USB driver
    dp800 = rm.open_resource(resource_name) # Open a resource that can comunicate with RIGOL PS
    # setup to starts the procedure
    dp800.write("SOUR1:VOLT 0")
    dp800.write("SOUR1:CURR 0")  
    sleep(0.1)
    dp800.write("SOUR2:VOLT 0")
    dp800.write("SOUR2:CURR 0")
    sleep(0.1)
    dp800.write("SOUR3:VOLT 0")
    dp800.write("SOUR3:CURR 0")
    sleep(0.1)
    
    dp800.write('OUTP CH1,ON')  # Enabling the Channel 1 output
    dp800.write('OUTP CH2,ON')  # Enabling the Channel 1 output
    dp800.write('OUTP CH3,ON')  # Enabling the Channel 1 output
    
    return rm, dp800
    
def closingPowerSupplyChannel(dp800, rm):
    #dp800.write('OUTP CH1,OFF')  # Enabling the Channel 1 output
    #dp800.write('OUTP CH2,OFF')  # Enabling the Channel 1 output
    #dp800.write('OUTP CH3,OFF')  # Enabling the Channel 1 output
    dp800.close()
    rm.close()
    
def setupMCU(serialObj, samplesPerMean, timeBetweenSamples):
    sleep(1)
    serialObj.write(samplesPerMean)
    sleep(1)
    serialObj.write(timeBetweenSamples)
    serialObj.reset_input_buffer()
    sleep(1)
    
def setPowerSupplyAllChCurrent(visaObj, Current):
    sendCommandPS(visaObj, 'SOUR1:CURR '+str(Current))
    sendCommandPS(visaObj, 'SOUR2:CURR '+str(Current))
    sendCommandPS(visaObj, 'SOUR3:CURR '+str(Current))
    
def setPowerSupplyAllChVoltage(visaObj, Voltage):
    sendCommandPS(visaObj, 'SOUR1:VOLT '+str(Voltage))
    sendCommandPS(visaObj, 'SOUR2:VOLT '+str(Voltage))
    sendCommandPS(visaObj, 'SOUR3:VOLT '+str(-Voltage))

def setPowerSupplyChVoltage(visaObj, Channel, Voltage):
    sendCommandPS(visaObj, 'SOUR'+str(Channel)+':VOLT '+str(Voltage))
    
def setPowerSupplyChCurrent(visaObj, Channel, Current, security = True):
    if security:
        if Current > 0.5:
            Current = 0.5
    sendCommandPS(visaObj, 'SOUR'+str(Channel)+':CURR '+str(Current))
    
def measurementNtransformation(serialObj, C):
    Field = getMeasurement(serialObj)
    return np.matmul(C, Field)
# -------------------------------------------------------------------------------------------------------------------