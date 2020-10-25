# -*- coding: utf-8 -*-
"""
Created on Wed Apr  3 16:55:50 2019

@author: aurelio
"""
import csv, matplotlib.pyplot as plt
from auxLib import getDataFromReader

#%% User code

csvPath = "./csvData/"
csvFileName = "fieldsNcurrents2 - Kopie"


#%%

fileField = csvPath + csvFileName + ".csv"
file = open(fileField, 'r')
reader = csv.reader(file, delimiter=';', quoting=csv.QUOTE_NONE)
floatData, channel, zeroField, ue, ue2 = getDataFromReader(reader, 1)

current = [list(),list(),list()]
aux = [0,0,0]
for i in range(0,len(floatData[3])):
    if channel[i] == 1:
        aux[0] = floatData[3][i]
    if channel[i] == 2:
        aux[1] = floatData[3][i]
    if channel[i] == 3:
        aux[2] = floatData[3][i]
        
    current[0].append(aux[0])
    current[1].append(aux[1])
    current[2].append(aux[2])
        



t = list(range(1,len(floatData[0])+1,1))

fig = plt.figure(figsize=(14,14), dpi=100)
fig.suptitle('Field x, y, z and current i', fontsize=20)

AXS = [fig.add_subplot(2,2,i, axisbelow=True) for i in range(1,5)]

subPlot = AXS

[[subPlot[i].set_xlabel('sample')] for i in range(0,4)]
subPlot[0].set_ylabel('x (uT)')
subPlot[1].set_ylabel('y (uT)')
subPlot[2].set_ylabel('z (uT)')
subPlot[3].set_ylabel('i (mA)')

[[subPlot[i].set_xlim([min(t)-1, max(t)+1])] for i in range(0,4)]
[[subPlot[i].set_ylim([min(floatData[i])-1,max(floatData[i])+1])] for i in range(0,4)]

subPlot[0].plot(t, floatData[0], color = 'r')
subPlot[1].plot(t, floatData[1], color = 'k')
subPlot[2].plot(t, floatData[2], color = 'm')
#subPlot[3].plot(t, floatData[3], color = 'b')
subPlot[3].plot(t, current[0], color = 'r')
subPlot[3].plot(t, current[1], color = 'k')
subPlot[3].plot(t, current[2], color = 'm')

fig.legend(["x", "y", "z","ix","iy","iz"])

fieldModules = list()

[[fieldModules.append(float((floatData[0][i]**2 + floatData[1][i]**2 + floatData[2][i]**2)**(1/2)))]for i in range(0, len(floatData[0]))]

fig3 = plt.figure(figsize = (12,12))
fig3.suptitle('Field module (uT) by sample', fontsize=20)

plt.plot(range(0,len(fieldModules)),fieldModules)










