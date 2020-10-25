# -*- coding: utf-8 -*-
"""
py file that automatic sets the field of the 3DHC
"""
#%% Imports

import numpy as np
import matplotlib.pyplot as plt
from time import sleep
import csv
from auxLib import  updateCoilPolarity, rotationMatrixCalculator, storeDataObj, writeDataOnFile, CurrentCalculatorObj, setField, startupCoilsPolarity, closeFile, closeSerialObj, createAndOpenFile, WhatTimeIsIt, makeDir, createSerialObj, COMdetect, changeCoilPolarity, measurementNtransformation, setPowerSupplyChVoltage, setPowerSupplyAllChVoltage, setPowerSupplyAllChCurrent, setupMCU, closingPowerSupplyChannel, startUpPowerSupply, getMeasurement, setPowerSupplyChCurrent

# To show numbers in console in the raw representation
np.set_printoptions(suppress=True)
#%% Channel controller class
#closeSerialObj(serialObj)
try:
#if 1:
    #%% User code
    # Setup information to the microcontroller, must be in binary format "b'information'"
    samplesPerMean = b'30'
    timeBetweenSamples = b'500'
    gain = 1
    samplingTime = 0.2*gain
    
    desiredField = [0, -4, -3]
    
    plotRealTime = 1
    
    meanSize = 1
    nSamples = 25
    plotLim = 30
    
    maxPoints = 30000
    
    #%% Start up code    
        
    # opening MCU comunication
    COM = COMdetect()

    serialObj = createSerialObj(COM)
    
    # Opening the MCU comunication channel
    setupMCU(serialObj, samplesPerMean, timeBetweenSamples)
    sleep(0.1)
    startupCoilsPolarity(serialObj)
    
    # Opening PS comunication channel
    resource_name = 'USB0::0x1AB1::0x0E11::DP8A203800261::0::INSTR'
    rm, dp800 = startUpPowerSupply(resource_name)


#%% Rotation matrix
    
    rotationMatrix, CurrentToField, FieldToCurrent, er = rotationMatrixCalculator(dp800, serialObj, [0.5,2.5,0.5])
    
    
#%% ClosedLoop control
    #--------------------------------------------------------------------------------------------------------------------------------------------
    
    dp800.write("DISP:MODE NORM")
    index = 0
    supLim = 500
    iAnt = [0, 0, 0]
    eAnt = [0, 0, 0]
    i = [0, 0, 0]
    e = [0, 0, 0]
    
    plotData = np.zeros((3,nSamples))
    dataIndex = 0
    t = range(0, nSamples)
    plt.figure(figsize=(14,14), dpi=100)
    
    fData = getMeasurement(serialObj)
    zeroField = np.matmul(rotationMatrix, fData)
    
    coilCurrentCalculatorObj = [(CurrentCalculatorObj(i+1, zeroField[i], FieldToCurrent[i], CurrentToField[i])) for i in range(0,3)]
    
    iAnt = [coilCurrentCalculatorObj[0].calculateCurrent(0), coilCurrentCalculatorObj[1].calculateCurrent(0), coilCurrentCalculatorObj[2].calculateCurrent(0)]
    
    while(index < maxPoints):
        
        
        for j in range(0,3):
            i[j] = eAnt[j]*samplingTime + iAnt[j]
            if i[j] > supLim:
                i[j] = supLim
            if i[j] < -supLim:
                i[j] = -supLim
                
        i[1] = eAnt[1]*samplingTime/6 + iAnt[1]
        if i[1] > supLim:
            i[1] = supLim
        if i[1] < -supLim:
            i[1] = -supLim  
        updateCoilPolarity(serialObj, i[0], 1)
        updateCoilPolarity(serialObj, i[1], 2)
        updateCoilPolarity(serialObj, i[2], 3)
        
        setPowerSupplyChCurrent(dp800, 1, float(i[0]/1000))
        setPowerSupplyChCurrent(dp800, 2, float(i[1]/1000))
        setPowerSupplyChCurrent(dp800, 3, float(i[2]/1000)) 
        
        fData = getMeasurement(serialObj)
        currentField = np.matmul(rotationMatrix, fData)
        
        for j in range(0,3):
            e[j] = desiredField[j] - currentField[j]
            eAnt[j] = e[j]
            iAnt[j] = i[j]
            
            
        if plotRealTime:
            
            
            for u in range(0,3):
                plotData[u][dataIndex] = 0
                for h in range(1,meanSize):
                    if dataIndex-h < 0:
                        plotData[u][dataIndex] += plotData[u][nSamples-h]
                    else:
                        plotData[u][dataIndex] += plotData[u][dataIndex-h]
                plotData[u][dataIndex] =  currentField[u]  
                plotData[u][dataIndex] = plotData[u][dataIndex]/meanSize
                
            ''' 
            plotData[0][dataIndex] = currentField[0]
            plotData[1][dataIndex] = currentField[1]
            plotData[2][dataIndex] = currentField[2]
            '''
            if index != 0:
                plt.clf()
                plt.ylim(-plotLim,plotLim)
                plt.plot(t, plotData[0], 'r', label = 'x axis')
                plt.plot(t, plotData[1], 'b', label = 'y axis')
                plt.plot(t, plotData[2], 'm', label = 'z axis')
                plt.legend()
                plt.pause(0.001)
                
            dataIndex += 1    
            
            if dataIndex >= nSamples:
                dataIndex = 0
                
                
        index += 1
      
    #%% Closing the resources and file
    
    closeSerialObj(serialObj)
    closingPowerSupplyChannel(dp800, rm)
    
    #%% Closing the resources and file if something goes wrong in the execution of the program
except Exception as error:

    print(error)
    
    closeSerialObj(serialObj)
    closingPowerSupplyChannel(dp800, rm)
    