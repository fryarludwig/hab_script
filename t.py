import serial
import math
import smbus
import os
import numpy
import RPi.GPIO as GPIO
import time

Vin = 3.3
Vsupply = 5
maxAD = 255

bus = smbus.SMBus(1)
address = 0x48

registerTrpi = 0x84  # ADC CH 0
registerText = 0xc4  # ADC CH 1
registerTbat = 0x94  # ADC CH 2
registerVbat = 0xd4  # ADC CH 3
registerRH   = 0xa4  # ADC CH 4
registerAccX = 0xe4  # ADC CH 5
registerAccY = 0xb4  # ADC CH 6
registerAccZ = 0xf4  # ADC CH 7

'''rawValPiTemp = bus.read_byte_data(address, registerTrpi)  # 4604
VTrpi = rawValPiTemp * Vin / maxAD
Rrpi = 51800 * ((Vin / VTrpi) - 1)
PiTempRHPurpose = (1 / ((.0014782389) + (.00023632193 * (math.log(Rrpi))) + ((.00000011403386 * (math.log(Rrpi))) ** 3)) - 273.15)
calculatedPiTemp = "%4.1f" % (1 / ((.0014782389) + (.00023632193 * (math.log(Rrpi))) + ((.00000011403386 * (math.log(Rrpi))) ** 3)) - 273.15)
fTrpi = open('sensorData/temp_raspi.txt', 'a')
fTrpi.write(str(VTrpi) + ' ' + calculatedPiTemp + '\n')
fTrpi.close()


rawValRH = bus.read_byte_data(address, registerRH)
VRH = rawValRH * 3.3 * ((10.1 + 38.9) / 38.9) / 255
RHApprox = ((VRH / Vsupply) - .16) / .0062
calculatedRHValue = "%4.1f" % ((RHApprox) / (1.0546 - (.00216 * PiTempRHPurpose)))
fRH = open('sensorData/humidity.txt', 'a')
fRH.write(str(rawValRH) + ' ' + calculatedRHValue + '\n')
fRH.close()
'''
rawValPiTemp = bus.read_byte_data(address, registerTrpi)  # 4604
VTrpi = rawValPiTemp * Vin / maxAD
Rrpi = 51800 * ((Vin / VTrpi) - 1)
TempRHPurpose = (1 / ((.0014782389) + (.00023632193 * (math.log(Rrpi))) + ((.00000011403386 * (math.log(Rrpi))) ** 3)) - 273.15)
calculatedPiTemp = "%4.1f" % (1 / ((.0014782389) + (.00023632193 * (math.log(Rrpi))) + ((.00000011403386 * (math.log(Rrpi))) ** 3)) - 273.15)
fTrpi = open('sensorData/temp_raspi.txt', 'a')
fTrpi.write(str(VTrpi) + ' ' + calculatedPiTemp + '\n')
fTrpi.close()

rawValRH = bus.read_byte_data(address, registerRH)
VRH = rawValRH * 3.3 * ((10.1 + 38.9) / 38.9) / 255
RHApprox = ((VRH / Vsupply) - .16) / .0062
calculatedRHValue = "%4.1f" % ((RHApprox) / (1.0546 - (.00216 * PiTempRHPurpose)))
fRH = open('sensorData/humidity.txt', 'a')
fRH.write(str(rawValRH) + ' ' + calculatedRHValue + '\n')
fRH.close()
