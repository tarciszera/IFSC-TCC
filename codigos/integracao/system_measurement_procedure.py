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
measurementPeriod = 0.01

csvPath = "./csvData/"
csvFileName = "fieldsNcurrents"

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
    
    while(1):
        if str(control[i]) == "['Current']":
            xablau = (currents[i]-currentOffset[channel[i]-1])
            
            #print(xablau)
            updateCoilPolarity(serialObj, xablau, channel[i])
            setPowerSupplyChCurrent(dp800, channel[i], float(xablau/1000))
            setPowerSupplyChVoltage(dp800, channel[i], 4)
        else:
            updateCoilPolarity(serialObj, currents[i], channel[i])
            setPowerSupplyChVoltage(dp800, channel[i], float(currents[i]/1000))
            setPowerSupplyChCurrent(dp800, channel[i], 0.5)
            
        sleep(measurementPeriod)
        input(i)  
        fData = getMeasurement(serialObj)
        fData = np.matmul(rotationMatrix, fData)
        nsei = 3
        for j in range(0,3):
            newMeasures[i][j] = fData[j]
            errors[i][j] = newMeasures[i][j] - floatData[j][i]
        i += indexSum
        
        if i > len(currents):
            break
        
        
        #if i == 4:
         #   indexSum = 1
        #if i == len(currents):
         #   i = 4
    
    print('absolute diference between measured field and expected field:')
    print(errors)
    print('absolute measured field:')
    print(newMeasures)
    
    closeSerialObj(serialObj)
    closingPowerSupplyChannel(dp800, rm)
    
except Exception as error:

    print(error)
    
    closeSerialObj(serialObj)
    closingPowerSupplyChannel(dp800, rm)