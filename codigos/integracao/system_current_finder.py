# -*- coding: utf-8 -*-
"""
py file that automatic sets the field of the 3DHC
"""
#%% Imports

import numpy as np
from time import sleep
import csv
from auxLib import  rotationMatrixCalculator, storeDataObj, writeDataOnFile, CurrentCalculatorObj, setField, startupCoilsPolarity, closeFile, closeSerialObj, createAndOpenFile, WhatTimeIsIt, makeDir, createSerialObj, COMdetect, changeCoilPolarity, measurementNtransformation, setPowerSupplyChVoltage, setPowerSupplyAllChVoltage, setPowerSupplyAllChCurrent, setupMCU, closingPowerSupplyChannel, startUpPowerSupply, getMeasurement, setPowerSupplyChCurrent

# To show numbers in console in the raw representation
np.set_printoptions(suppress=True)
#%% Channel controller class
#closeSerialObj(serialObj)
try:
#if 1:
    #%% User code
    # Setup information to the microcontroller, must be in binary format "b'information'"
    samplesPerMean = b'4'
    timeBetweenSamples = b'50'
    
    samplingTime = 0.1
    gain = 4
    
    fileDir = "./csvData/"
    fileName = 'fieldsNcurrents'
    
    # Magnetic field in uT
    pattern = 'straight'
    #pattern = 'Circular'
    lowerFieldLimit = -200
    upperFieldLimit = 200
    fieldResolution = 10
    
#%% Start up code    
        
    steps = round((upperFieldLimit - lowerFieldLimit)/fieldResolution) + 1
    
    # Teorethical values for field/current ration
    xTeo = 0.5
    yTeo = 2.5
    zTeo = 0.5
    # opening MCU comunication
    COM = COMdetect()

    serialObj = createSerialObj(COM)
    
    # Opening the MCU comunication channel
    setupMCU(serialObj, samplesPerMean, timeBetweenSamples)
    sleep(0.1)
    startupCoilsPolarity(serialObj)
    
    # Opening PS comunication channel
    resource_name = 'USB0::0x1AB1::0x0E11::DP8A203800261::0::INSTR'   
    # specific for this dp800, you can find to another using
    '''
    rm = visa.ResourceManager() # To connect the wraper to the USB driver
    print(rm.list_resources_info()) # to take the resource name
    '''
    
    # Creating VISA objects
    rm, dp800 = startUpPowerSupply(resource_name)


#%% Rotation matrix
    
    rotationMatrix, CurrentToField, FieldToCurrent, er = rotationMatrixCalculator(dp800, serialObj, [xTeo,yTeo,zTeo])
    
    print("field/current ration error (%) for each coil")
    print(er)
    
    
#%% Setup of current manangers
    fData = getMeasurement(serialObj)
    zeroField = np.matmul(rotationMatrix, fData)
    
    coilCurrentCalculatorObj = [(CurrentCalculatorObj(i+1, zeroField[i], FieldToCurrent[i], CurrentToField[i])) for i in range(0,3)]
    
#%% Setting the desired field
    dataManager = storeDataObj(rotationMatrix)
    
    for i in range(2,-1,-1):
        fieldMeasurement, current, control = setField(dp800, serialObj, coilCurrentCalculatorObj[i], samplingTime*gain, float(0), rotationMatrix)
        dataManager.storeData(fieldMeasurement, current, i+1,zeroField[i], control)
        
        
    print("zeroField :\n"+str(zeroField))
    print("fieldMeasurement : \n"+str(fieldMeasurement))
    #-----------
    
    if pattern == 'straight':
        innerCoilFieldVector = np.linspace(lowerFieldLimit, upperFieldLimit, steps)    
        for j in range(0, len(innerCoilFieldVector)):
            # Send the command to set the current
            fieldMeasurement, current, control = setField(dp800, serialObj, coilCurrentCalculatorObj[1], samplingTime*gain, float(innerCoilFieldVector[j]), rotationMatrix, breakCondition = float(innerCoilFieldVector[j]/500), maxIterations = 40)
            dataManager.storeData(fieldMeasurement, current, 2,zeroField[1], control)
            
            print("fieldMeasurement : \n"+str(fieldMeasurement))
            
            if (innerCoilFieldVector[j] != 0):
                fieldError = np.linalg.norm((fieldMeasurement[1] - innerCoilFieldVector[j])/innerCoilFieldVector[j])*100
            else:
                fieldError = float('inf')
            print("error (%) between desired and achieved field: \n"+str(fieldError))
    elif pattern == 'Circular':
        radius = 200
        size = 200
        teta = np.linspace(0,2*np.pi,size)
        innerCoilFieldVector = np.sin(teta)*radius
        midCoilFieldVector = np.cos(teta)*radius
        
        for j in range(0, len(innerCoilFieldVector)):
            # Send the command to set the current

            fieldMeasurement, current, control = setField(dp800, serialObj, coilCurrentCalculatorObj[1], samplingTime*gain, float(innerCoilFieldVector[j]), rotationMatrix, breakCondition = 1, maxIterations = 40)
            fieldMeasurement2, current2, control2 = setField(dp800, serialObj, coilCurrentCalculatorObj[2], samplingTime*gain, float(midCoilFieldVector[j]), rotationMatrix, breakCondition = 1, maxIterations = 40)
            
            dataManager.storeData(fieldMeasurement, current, 2,zeroField[1], control)
            dataManager.storeData(fieldMeasurement2, current2, 3,zeroField[2], control2)
            
            print("fieldMeasurement : \n"+str(fieldMeasurement))
            
    else:
        print("Please choose a existing parttern")
    #------------
        
    dataManager.createCSVfile(fileDir, fileName)
    
    #%% Closing the resources and file
    
    closeSerialObj(serialObj)
    closingPowerSupplyChannel(dp800, rm)
    
    #%% Closing the resources and file if something goes wrong in the execution of the program
except Exception as error:

    print(error)
    
    closeSerialObj(serialObj)
    closingPowerSupplyChannel(dp800, rm)
    