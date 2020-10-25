AllCoilsWireDiameter = (0.56 + 0.047)*1.09   # Wire diameter in mm (from dataSheet)

minRes = 0.06736                            # Ohms/m (from dataSheet)
nominal = 0.06940                           # Ohms/m (from dataSheet)
maxRes = 0.07153                            # Ohms/m (from dataSheet)

wireResistence = maxRes                     # Ohms/m (from dataSheet)


# z First Coil configuration

zInitialValueCoilDiameter = 80 # Initial value for the function that calculates the precise diameter


zCoilCurrent = 0.5             # Amperes
zLoopsInEachEvenLayer = 10     # The number of loops in each layer
zEvenLayers = 7                # The evenlayers always have to be equal to oddLayers or oddLayers + 1
zOddLayers = 7
zDesiredField = 1.25           # The desired field in the center of the Helmholtz coil

# y Second Coil configuration

yCoilCurrent = 0.5             # Amperes
yLoopsInEachEvenLayer = 6      # The number of loops in each layer
yEvenLayers = 3                # The evenlayers always have to be oddLayers + 1 or equal oddLayers
yOddLayers = 3
yDesiredField = 0.25           # The desired field in the center of the Helmholtz coils

# x Third Coil configuration

xCoilCurrent = 0.5             # Amperes
xLoopsInEachEvenLayer = 7      # The number of loops in each layer
xEvenLayers = 3                # The evenlayers always have to be oddLayers + 1 or equal oddLayers
xOddLayers = 3
xDesiredField = 0.25           # The desired field in the center of the Helmholtz coil