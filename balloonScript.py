"""

Changelog:
07/14/15 - KFL: Merged balloon script files, removed unused functions
				Refactored serial input/output. Changes not tested.
				
"""

from __future__ import print_function

import serial
import math
import smbus
import RPi.GPIO as GPIO
import time
from time import sleep
import os
import numpy

RADIO_CALLSIGN = "HAB"
RADIO_SERIAL_PORT = "/dev/ttyUSB0" # COMX on Windows, /dev/RADIO_SERIAL_PORT on Linux
RADIO_BAUDRATE = 9600

GPS_LOG_FILE_LOCATION = r"logData/gps_log.txt"
SCRIPT_LOG_FILE_LOCATION = r"logData/balloon_script_log.txt"
TELEMETRY_FILE_BASE = r"logData/dictionary.txt"

GPS_SERIAL_PORT = "/dev/ttyAMA0"
GPS_BAUDRATE = 4800
GPS_LEN_IN_BYTES = 75

SHORT_SLEEP_DURATION = 0.05
LONG_SLEEP_DURATION = 1
HEARTBEAT_INTERVAL = 100

Vin = 3.3
maxAD = 255

bus = smbus.SMBus(1)
address = 0x48

registerTrpi = 0x84 #ADC CH 0
registerText = 0xc4 #ADC CH 1
registerTbat = 0x94 #ADC CH 2
registerVbat = 0xd4 #ADC CH 3
registerRH   = 0xa4 #ADC CH 4
	
"""
TODO:
-Refactor serial in and out to match MoGS implementation
-Change registerVbat to ADC CH 3

"""
class balloonScript():
	def __init__(self):
		# Open output files
		self.scriptLog = open(SCRIPT_LOG_FILE_LOCATION, "a")
		self.gpsLog = open(GPS_LOG_FILE_LOCATION, "a")
		
		try:
			self.fTrpi = open('temp_raspi.txt','a')
		except:
			self.fTrpi = open('temp_raspi.txt','w')
		try:
			self.fText = open('temp_external.txt','a')
		except:
			self.fText = open('temp_external.txt','w')
		try:
			self.fTbat = open('temp_batteries.txt','a')
		except:
			self.fTbat = open('temp_batteries.txt','w')
		try:
			self.fVbat = open('voltage_batteries.txt','a')
		except:
			self.fVbat = open('voltage_batteries.txt','w')
		try:
			self.fRH = open('humidity.txt','a')
		except:
			self.fRH = open('humidity.txt','w')
		
		# Open serial ports
		self.radioSerialPort = None
		self.gpsSerialPort = None
		
		self.openRadioSerialPort()
		self.openGpsSerialPort()
		
		self.runBalloonScript()
		
	def runBalloonScript(self):
		messageToSend = "NULL_MESSAGE"
		
		while(True):
			# send
			try:
				messageToSend = self.formatData(self.gpsSerialInput(), self.getSensorData())
				if (messageToSend != "INVALID DATA"):
					print("Sending: " + messageToSend)
	
					self.sendSerialOutput(messageToSend)
				self.gpsLog.writelines(messageToSend)
				print("Serial: " + messageToSend)
			except:
                                self.openRadioSerialPort()
                                self.openGpsSerialPort()
				print("Caught exception in main loop.")
				
			self.scriptLog.write(messageToSend)
			
			# receive
			messageReceived = self.radioSerialInput()
			
			# act on receive
			self.handleMessage(messageReceived)
	
			self.scriptLog.writelines(messageReceived)
		
	def handleMessage(self, message):
		for line in message:
			if not ("No messages received." in line):
				if ("releaseBalloonNow" in line):
					self.releaseBalloon()
					break
				else:
					print("Received unknown: {0}".format(line))
					print("Saving message to file")
					print("Sending NO ACK message back to GS")
	
	def formatData(self, gpsString, dataString):
		finalDataString = "INVALID DATA"
		currGpsString = ""
		
		try:
			gpsSplit = gpsString.split(",")
			currGpsString = "{},{},{},{}".format(gpsSplit[1][:6],
												gpsSplit[2],
												gpsSplit[4],
												gpsSplit[9])
	
		except:
			currGpsString = "0,0,0,0"
	
		if (dataString == "NO DATA" and gpsSplit == "0,0,0,0"):
			int ("INVALID DATA STRINGS GIVEN")
		else:
			finalDataString = currGpsString + dataString
	
		return finalDataString
	
	def getSensorData(self):
		calculatedPiTemp = 0
		calculatedExternalTemp = 0
		calculatedBatteryTemp = 0
		calculatedVoltageBattery = 0
		
		try:
			rawValPiTemp = bus.read_byte_data(address, registerTrpi) #4604
			VTrpi = rawValPiTemp * Vin / maxAD
			Rrpi = 51800 * ((Vin/VTrpi) - 1)
			calculatedPiTemp = "%4.1f" % (1/((.0014782389) + (.00023632193 * (math.log(Rrpi))) + ((.00000011403386 * (math.log(Rrpi))) ** 3)) - 273.15)
			self.fTrpi = open('temp_raspi.txt','a')
			self.fTrpi.write(str(VTrpi) + ' ' + calculatedPiTemp + '\n')
			self.fTrpi.close()
		except:
			calculatedPiTemp = "NO_VAL"
		
		try:
			rawValExternalTemp = bus.read_byte_data(address, registerText) #4599
			VText = rawValExternalTemp * 3.3 / 255
			Rext = 51800 * ((Vin/VText) - 1)
			calculatedExternalTemp = "%4.1f" % (1/((.0014732609) + (.00023727640 * (math.log(Rext))) + ((.00000010814580 * (math.log(Rext))) ** 3)) - 273.15)
			self.fText = open('temp_external.txt','a')
			self.fText.write(str(VText) + ' ' + calculatedExternalTemp + '\n')
			self.fText.close()
		except:
			calculatedExternalTemp = "NO_VAL"
		
		try:
			rawValBatteryTemp = bus.read_byte_data(address, registerTbat) #4600
			VTbat = rawValBatteryTemp * 3.3 / 255
			Rbat = 51800 * ((Vin/VTbat) - 1)
			calculatedBatteryTemp = "%4.1f" % ((1/((.0014721232) + (.00023728796 * (math.log(Rbat))) + ((.00000010792173 * (math.log(Rbat))) ** 3)) - 273.15))
			self.fTbat = open('temp_batteries.txt','a')
			self.fTbat.write(str(VTbat) + ' ' + calculatedBatteryTemp + '\n')
			self.fTbat.close()
		except:
			calculatedBatteryTemp = "NO_VAL"
		
		try:
			rawValBatteryVoltage = bus.read_byte_data(address, registerVbat)
			calculatedVoltageBattery = "%4.1f" % (.05155 * rawValBatteryVoltage + .18659)
			self.fVbat = open('voltage_batteries.txt','a')
			self.fVbat.write(str(rawValBatteryVoltage) + ' ' + calculatedVoltageBattery + '\n')
			self.fVbat.close()
		except:
			calculatedVoltageBattery = "NO_VAL"
                
	
		return ",{},{},{},{}".format(calculatedPiTemp, calculatedExternalTemp, calculatedBatteryTemp, calculatedVoltageBattery)
			
	def gpsSerialInput(self):
		messageReceived = "NO_GPS_DATA\n"
		serialInput = ""
		retries = 5
		iterationsToWait = 100
		
		try:
			while (retries > 0 and iterationsToWait > 0):
				if (self.gpsSerialPort.inWaiting() > 0):				# If there's a buffer for us to read
					serialInput = self.gpsSerialPort.readline(1024)
					if (serialInput[:6] == r"$GPGGA"):	# Makes sure this is the line we want
						break							# This is our stop
					else:
						#print("Discarding unused data: " + serialInput)
						serialInput = ""				# This is not the data we're looking for
						retries -= 1
				else:
					iterationsToWait -= 1
			
		except:
			print("Unable to read serial input: {0} at baud {1}".format(GPS_SERIAL_PORT, GPS_BAUDRATE))
		
		if (retries > 0 and iterationsToWait > 0):		# We found what we wanted
			messageReceived = serialInput
		
		return messageReceived
	
	
	def radioSerialInput(self):
		serialInput = ""
		
		try:
			if not (self.radioSerialPort.inWaiting()):
				sleep(1)
			while(self.radioSerialPort.inWaiting()):
				serialInput += self.radioSerialPort.readline()
			print(len("Serial Input: " + serialInput))
		except:
			print("Unable to open serial port for input on " + RADIO_SERIAL_PORT)
			
		return serialInput
	
	def sendSerialOutput(self, line):
		try:
                        print(RADIO_CALLSIGN + "," + line + ",END_TX\n")
			line = self.radioSerialPort.write(RADIO_CALLSIGN + "," + line + ",END_TX\n")
		except:
			print("Unable to write to serial port on " + RADIO_SERIAL_PORT)
	
	def openRadioSerialPort(self):
		try:
			self.radioSerialPort.close()
		except:
			print("Unable to close serial port " + RADIO_SERIAL_PORT)
			
		try:
			self.radioSerialPort=serial.Serial(port = RADIO_SERIAL_PORT, baudrate = RADIO_BAUDRATE, timeout = 2)
		except:
			print("Unable to open GPS serial port")
	
	def openGpsSerialPort(self):
		try:
			self.gpsSerialPort.close()
		except:
			print("Unable to close serial port " + GPS_SERIAL_PORT)
			
		try:
			self.gpsSerialPort=serial.Serial(port = GPS_SERIAL_PORT, baudrate = GPS_BAUDRATE, timeout = 2)
		except:
			print("Unable to open GPS serial port")
	
	def releaseBalloon(self):
		sleep(4)
		print("Activating balloon release mechanism")
		print("BRM is activated")
		self.sendSerialOutput("BRM is activated")
		print("BRM successfully released")
		self.sendSerialOutput("BRM is released")
		self.sendSerialOutput("MOCK: BRM Activated")
	#	GPIO.setmode(GPIO.BOARD)
	#	
	#	GPIO.setup(11, GPIO.OUT)
	#	
	#	GPIO.output(11,0)
	#	
	#	password = raw_input()
	#	
	#	while True:
	#		if str(password) == 'SSAGhabRELEASE':
	#			zero = time.time()
	#			while time.time() - zero < 9001:
	#				GPIO.output(11,1)
	#			break
	#		else:
	#			password = raw_input()
	#	GPIO.cleanup()
		
		'''
		NOTES
		
		Orange wire must be connected to pin 5 of MOSFET
		Yellow wire must be connected to pin 7 of MOSFET
		If raspi pin is high, motor will retract
		If raspi pin is low, motor will expand
		
		'''
	
	def transmitPicture(self):
	#	Get picture
	#		ls -> get last item
	#		Send item
	#			Timeout - 10s
	#		Return to normal broadcast
		return None
	
	def reportStateOfHealth(self):
	#	Diagnose errors
	#		Check for radios on network
	#		Report filesystem
	#		Return log
	#		Return disk usage
	#		Return battery usage
	#		Return processor / memory usage
	#		Return online components list
		return None

if __name__ == '__main__':
	runScript = balloonScript()
	
	
