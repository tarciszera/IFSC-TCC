# -*- coding: utf-8 -*-
"""
Created on Thu May 16 13:18:26 2019

@author: CTR Magnetic Demo
"""

# -*- coding: utf-8 -*-
"""
Created on Mon May 13 13:02:43 2019

@author: CTR Magnetic Demo
"""

#%% Imports

import numpy as np
import csv
from time import sleep
from auxLib import  updateCoilPolarity, getDataFromReader, rotationMatrixCalculator, strToFloat, storeDataObj, writeDataOnFile, CurrentCalculatorObj, setField, startupCoilsPolarity, closeFile, closeSerialObj, createAndOpenFile, WhatTimeIsIt, makeDir, createSerialObj, COMdetect, changeCoilPolarity, measurementNtransformation, setPowerSupplyChVoltage, setPowerSupplyAllChVoltage, setPowerSupplyAllChCurrent, setupMCU, closingPowerSupplyChannel, startUpPowerSupply, getMeasurement, setPowerSupplyChCurrent

#%% User code

samplesPerMean = b'2'
timeBetweenSamples = b'100'
measurementPeriod = 0.00

csvPath = "./csvData/"
csvFileName = "fieldsNcurrents2"

#%%

# To show numbers in console in the raw representation
np.set_printoptions(suppress=True)

fileField = csvPath + csvFileName + ".csv"

file = open(fileField, 'r')
reader = csv.reader(file, delimiter=';', quoting=csv.QUOTE_NONE)
floatData, channel, zeroField, rotationMatrixMeasures, control = getDataFromReader(reader, 1)
file.close()


#%% Start up code    
#closeSerialObj(serialObj)
# Teorethical values for field/current ration
teorethicalRelationship = [0.5, 2.5, 0.5]


try:
    # opening MCU comunication
    COM = COMdetect()
    
    serialObj = createSerialObj(COM)
        
    # Opening the MCU comunication channel
    setupMCU(serialObj, samplesPerMean, timeBetweenSamples)
    sleep(0.1)
    startupCoilsPolarity(serialObj)
    
    resource_name = 'USB0::0x1AB1::0x0E11::DP8A203800261::0::INSTR'       
    rm, dp800 = startUpPowerSupply(resource_name)
    
    rotationMatrix = rotationMatrixMeasures
       
    # comparing the zero fields
    setPowerSupplyAllChVoltage(dp800, 4)
    setPowerSupplyAllChCurrent(dp800, 0)
    
    fData = getMeasurement(serialObj)
    zeroFieldMeasured = np.matmul(rotationMatrix, fData)
    
    error = np.zeros((3,1))
    for i in range(0,3):
        error [i]= (zeroFieldMeasured[i] - zeroField[i])
    print("zero field absolute diference:")
    print(error)
    
    currentOffset = [0,0,0]
    
    for i in range(0,3):
        currentOffset[i] = 0#error[i]/teorethicalRelationship[i]    
            
    currents = floatData[3]
    newMeasures = np.zeros((len(currents),3))
    errors= np.zeros((len(currents),3))
    dp800.write("DISP:MODE NORM")
    sleep(0.01)
    i = 0
    indexSum = 1
    xablau = 90
    flag = 1
    time = 0.12
    a = 0
    b = 0
    FieldToCurrent = [0.5, 2.5, 0.5]
    rotationMatrix, CurrentToField, FieldToCurrent, er = rotationMatrixCalculator(dp800, serialObj, [0.5,2.5,0.5])
    
    
    for j in range(0,3):
        updateCoilPolarity(serialObj, currents[j], channel[j])
        dp800.write('SOUR'+str(int(channel[j]))+':CURR '+str(float(currents[j]/1000)))
        
    coilCurrentCalculatorObj = [(CurrentCalculatorObj(i+1, zeroFieldMeasured[i], FieldToCurrent[i], CurrentToField[i])) for i in range(0,3)]
    time = 0.12
    i = 0
    maxTurns = 30000
    while(i < maxTurns):
        
        xablau2 = xablau + currents[1]
        updateCoilPolarity(serialObj, xablau2, 2)
        dp800.write('SOUR'+str(int(2))+':CURR '+str(float(xablau2/1000)))
        dp800.write('SOUR'+str(int(3))+':CURR '+str(float(0/1000)))
        sleep(time)
        
        xablau3 = xablau*5 + currents[0]
        updateCoilPolarity(serialObj, xablau3, 3)
        dp800.write('SOUR'+str(int(3))+':CURR '+str(float(xablau3/1000)))
        dp800.write('SOUR'+str(int(2))+':CURR '+str(float(0/1000)))
        sleep(time)
        
        xablau2 = -xablau + currents[1]
        updateCoilPolarity(serialObj, xablau2, 2)
        dp800.write('SOUR'+str(int(2))+':CURR '+str(float(xablau2/1000)))
        dp800.write('SOUR'+str(int(3))+':CURR '+str(float(0/1000)))
        sleep(time)
        
        xablau3 = -xablau*5 + currents[0]
        updateCoilPolarity(serialObj, xablau3, 3)
        dp800.write('SOUR'+str(int(3))+':CURR '+str(float(xablau3/1000)))
        dp800.write('SOUR'+str(int(2))+':CURR '+str(float(0/1000)))
        sleep(time)
        
        b = a
        a = WhatTimeIsIt()
        a = float(a[18:23])
        #print(a-b)
        i+=1
    '''    
    fieldMeasurement, current2, control = setField(dp800, serialObj, coilCurrentCalculatorObj[1], 0.5, float(4), rotationMatrix, breakCondition = 0.5, maxIterations = 40)
    fieldMeasurement, current3, control = setField(dp800, serialObj, coilCurrentCalculatorObj[2], 0.5, float(3.6), rotationMatrix, breakCondition = 0.5, maxIterations = 40)
    print(current2)
    print(current3)
    print(fieldMeasurement)
    '''    
    i2 = coilCurrentCalculatorObj[1].calculateCurrent(4.8)
    i3 = coilCurrentCalculatorObj[2].calculateCurrent(2)
    updateCoilPolarity(serialObj, i2, 2)
    updateCoilPolarity(serialObj, i3, 3)
    dp800.write('SOUR'+str(int(3))+':CURR '+str(float(i3/1000)))
    dp800.write('SOUR'+str(int(2))+':CURR '+str(float(i2/1000)))
    
    print(i2)
    print(i3)
    closeSerialObj(serialObj)
    closingPowerSupplyChannel(dp800, rm)
    
except Exception as error:

    print(error)
    
    closeSerialObj(serialObj)
    closingPowerSupplyChannel(dp800, rm)