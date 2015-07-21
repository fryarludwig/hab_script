from __future__ import print_function

"""

Changelog:
07/14/15 - KFL: Merged balloon script files, removed unused functions
				Refactored serial input/output. Changes not tested.
07/14/15 - KFL: Updated a few things, changed GPS parsing
"""

"""

Misc. Notes:

- If i2c directory is not found:
	- Enable through raspi-config
	- Edit /etc/modules and add these two lines: "i2c-bcm2708" and "i2c-dev"
	- Reboot
- GPS via UART freezes - unsure as to the reason why
	- Currently running GPS via USB, and this works flawlessly

"""

import serial
import math
import smbus
import os
import numpy
import RPi.GPIO as GPIO
import time
import cv2
import datetime

RADIO_CALLSIGN = "HAB"
RADIO_SERIAL_PORT = "/dev/ttyUSB0"  # COMX on Windows, /dev/RADIO_SERIAL_PORT on Linux
RADIO_BAUDRATE = 9600

GPS_LOG_FILE_LOCATION = r"logData/gps_log.txt"
RADIO_LOG_FILE_LOCATION = r"logData/radio_log.txt"
SCRIPT_LOG_FILE_LOCATION = r"logData/balloon_script_log.txt"

GPS_SERIAL_PORT = "/dev/ttyUSB1"
GPS_BAUDRATE = 4800

RELEASE_BALLOON_ALTITUDE = 23774  # 78,000 feet

Vin = 3.3
Vsupply = 5
maxAD = 255

bus = smbus.SMBus(1)
address = 0x48

registerTrpi = 0x84  # ADC CH 0
registerText = 0xc4  # ADC CH 1
registerTbat = 0x94  # ADC CH 2
registerVbat = 0xd4  # ADC CH 3
registerRH = 0xa4  # ADC CH 4
registerAccX = 0xe4  # ADC CH 5
registerAccY = 0xb4  # ADC CH 6
registerAccZ = 0xf4  # ADC CH 7

snapCount = 0
i = 0

cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc('I', 'Y', 'U', 'V')
out = cv2.VideoWriter('output.avi', fourcc, 25.0, (640, 480))

if cap.isOpened() == False:
	cap.open(0)

class balloonScript():
	def __init__(self):
		# Open output files

		try:
			self.scriptLog = open(SCRIPT_LOG_FILE_LOCATION, "a")
		except:
			self.gpsLog = open(SCRIPT_LOG_FILE_LOCATION, "w")
		try:
			self.gpsLog = open(GPS_LOG_FILE_LOCATION, "a")
		except:
			self.gpsLog = open(GPS_LOG_FILE_LOCATION, "w")
		try:
			self.fTrpi = open('sensorData/temp_raspi.txt', 'a')
		except:
			self.fTrpi = open('sensorData/temp_raspi.txt', 'w')
		try:
			self.fText = open('sensorData/temp_external.txt', 'a')
		except:
			self.fText = open('sensorData/temp_external.txt', 'w')
		try:
			self.fTbat = open('sensorData/temp_batteries.txt', 'a')
		except:
			self.fTbat = open('sensorData/temp_batteries.txt', 'w')
		try:
			self.fVbat = open('sensorData/voltage_batteries.txt', 'a')
		except:
			self.fVbat = open('sensorData/voltage_batteries.txt', 'w')
		try:
			self.fRH = open('sensorData/humidity.txt', 'a')
		except:
			self.fRH = open('sensorData/humidity.txt', 'w')
		try:
			self.fAcc = open('sensorData/accelerometer.txt', 'a')
		except:
			self.fAcc = open('sensorData/accelerometer.txt', 'w')

		# Set up variables to be used by the script
		self.altitudeDataList = []
		self.balloonReleaseArmed = False
		self.balloonReleaseActivated = False

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
				gpsMessage, validGpsData = self.processGpsData(self.gpsSerialInput())
				sensorData, validSensorData = self.getSensorData()

				if (validGpsData or validSensorData):
					self.sendSerialOutput(validGpsData + validSensorData)

				self.gpsLog.writelines(messageToSend)

			except:
				print("Caught exception in main loop.")
				return 0

			self.scriptLog.write(messageToSend)

			# receive
			messageReceived = self.radioSerialInput()

			# act on receive
			self.handleMessage(messageReceived)

			self.scriptLog.writelines(messageReceived)

	def handleMessage(self, message):
		try:
			for line in message.split(',END_TX\n'):
				if (len(line) > 0):
					if (line[:3] == "nps"):
						if (line[4:7] == "cmd"):
							self.processCommand(line[8:])

					elif (line[:6] == "chase1" or
						  line[:6] == "chase2" or
						  line[:6] == "chase3"):
						if (line[7:10] == "cmd"):
							self.processCommand(line[11:])

				logRadioMessage(line)
		except:
			print("Exception in handling balloon release")

	def processCommand(self, command):
		if (command == "ARM_BRM"):
			self.balloonReleaseArmed = True
			self.sendSerialOutput("BRM_ARMED")
		elif (command == "DISARM_BRM"):
			self.balloonReleaseArmed = True
			self.sendSerialOutput("BRM_DISARMED")
		elif (command == "SSAG_RELEASE_BALLOON" and
				self.balloonReleaseArmed):
			self.releaseBalloon()
			self.sendSerialOutput("BRM_ACTIVATED")
		elif (command == "RESET_BRM"):
			self.balloonReleaseArmed = False
			self.brmReset()
			self.sendSerialOutput("BRM_RESET")
		else:
			self.sendSerialOutput("NO_OP")

	def getSensorData(self):
		calculatedPiTemp = 0
		calculatedExternalTemp = 0
		calculatedBatteryTemp = 0
		calculatedVoltageBattery = 0
		calculatedRHValue = 0
		rawAccelX = 0
		rawAccelY = 0
		rawAccelZ = 0

		try:
			rawValPiTemp = bus.read_byte_data(address, registerTrpi)  # 4604
			VTrpi = rawValPiTemp * Vin / maxAD
			Rrpi = 51800 * ((Vin / VTrpi) - 1)
			PiTempRHPurpose = (1 / ((.0014782389) + (.00023632193 * (math.log(Rrpi))) + ((.00000011403386 * (math.log(Rrpi))) ** 3)) - 273.15)
			calculatedPiTemp = "%4.1f" % (1 / ((.0014782389) + (.00023632193 * (math.log(Rrpi))) + ((.00000011403386 * (math.log(Rrpi))) ** 3)) - 273.15)
			self.fTrpi = open('sensorData/temp_raspi.txt', 'a')
			self.fTrpi.write(str(VTrpi) + ' ' + calculatedPiTemp + '\n')
			self.fTrpi.close()
		except:
			calculatedPiTemp = "NO_VAL"

		try:
			rawValExternalTemp = bus.read_byte_data(address, registerText)  # 4599
			VText = rawValExternalTemp * 3.3 / 255
			Rext = 51800 * ((Vin / VText) - 1)
			calculatedExternalTemp = "%4.1f" % (1 / ((.0014732609) + (.00023727640 * (math.log(Rext))) + ((.00000010814580 * (math.log(Rext))) ** 3)) - 273.15)
			self.fText = open('sensorData/temp_external.txt', 'a')
			self.fText.write(str(VText) + ' ' + calculatedExternalTemp + '\n')
			self.fText.close()
		except:
			calculatedExternalTemp = "NO_VAL"

		try:
			rawValBatteryTemp = bus.read_byte_data(address, registerTbat)  # 4600
			VTbat = rawValBatteryTemp * 3.3 / 255
			Rbat = 51800 * ((Vin / VTbat) - 1)
			calculatedBatteryTemp = "%4.1f" % ((1 / ((.0014721232) + (.00023728796 * (math.log(Rbat))) + ((.00000010792173 * (math.log(Rbat))) ** 3)) - 273.15))
			self.fTbat = open('sensorData/temp_batteries.txt', 'a')
			self.fTbat.write(str(VTbat) + ' ' + calculatedBatteryTemp + '\n')
			self.fTbat.close()
		except:
			calculatedBatteryTemp = "NO_VAL"

		try:
			rawValBatteryVoltage = bus.read_byte_data(address, registerVbat)
			calculatedVoltageBattery = "%4.1f" % (.05155 * rawValBatteryVoltage + .18659)
			self.fVbat = open('sensorData/voltage_batteries.txt', 'a')
			self.fVbat.write(str(rawValBatteryVoltage) + ' ' + calculatedVoltageBattery + '\n')
			self.fVbat.close()
		except:
			calculatedVoltageBattery = "NO_VAL"

		try:
			rawValRH = bus.read_byte_data(address, registerRH)
			VRH = rawValRH * 3.3 * ((10.1 + 38.9) / 38.9) / 255
			RHApprox = ((VRH / Vsupply) - .16) / .0062
			calculatedRHValue = "%4.1f" % ((RHApprox) / (1.0546 - (.00216 * PiTempRHPurpose)))
			self.fRH = open('sensorData/humidity.txt', 'a')
			self.fRH.write(str(rawValRH) + ' ' + calculatedRHValue + '\n')
			self.fRH.close()
		except:
			calculatedRHValue = "NO_VAL"

		try:
			rawAccelX = str(bus.read_byte_data(address, registerAccX))
			rawAccelY = str(bus.read_byte_data(address, registerAccY))
			rawAccelZ = str(bus.read_byte_data(address, registerAccZ))
			self.fAcc = open('sensorData/accelerometer.txt', 'a')
			self.fAcc.write(rawAccelX + ' ' + rawAccelY + ' ' + rawAccelZ + '\n')
			self.fAcc.close()
		except:
			rawAccelX = "NO_VAL"
			rawAccelY = "NO_VAL"
			rawAccelZ = "NO_VAL"


		return ",{},{},{},{},{},{},{},{}".format(calculatedPiTemp, calculatedExternalTemp,
												calculatedBatteryTemp, calculatedVoltageBattery,
												calculatedRHValue, rawAccelX, rawAccelY, rawAccelZ)

	def processAltitude(self, currAlt):
		if (currAlt > RELEASE_BALLOON_ALTITUDE):
			self.releaseBalloon()
		else:
			try:
				balloonIsFalling = True

				numOfDataPoints = len(self.altitudeDataList)

				if (numOfDataPoints > 0):
					self.altitudeDataList.append(currAlt - self.altitudeDataList[-1])  # How much balloon has risen since last check
				else:
					self.altitudeDataList.append(currAlt)

				if (numOfDataPoints > 10):  # Only keep latest 10 height differences
					self.altitudeDataList.pop(0)

					for deltaAlt in self.altitudeDataList:  # Only check if we have enough data points
						if (deltaAlt > 0):
							balloonIsFalling = False
							break
					if (balloonIsFalling):
						self.releaseBalloon()

			except:
				print("Error in calculating altitude change")

	def processGpsData(self, gpsString):
		validGpsData = False
		formattedGpsString = 'NO_VAL,NO_VAL,NO_VAL,NO_VAL'

		if (gpsString != "NO_GPS_DATA\n"):
			try:
				gpsSplit = gpsString.split(",")
				if (len(gpsSplit) == 15):
					time = gpsSplit[1][:6]

					latitude = gpsSplit[2]
					if(len(latitude) > 0):
						degrees = float(latitude[:2])
						minutes = float(latitude[2:])

						if (gpsSplit[3] == "S"):
							latitude = "%4.5f" % (-1 * (degrees + (minutes / 60)))
						else:
							latitude = "%4.5f" % (degrees + (minutes / 60))
					else:
						latitude = ''

					longitude = gpsSplit[4]
					if(len(longitude) > 0):
						degrees = float(longitude[:3])
						minutes = float(longitude[3:])
						if (gpsSplit[5] == "W"):
							longitude = "%4.5f" % (-1 * (degrees + (minutes / 60)))
						else:
							longitude = "%4.5f" % (degrees + (minutes / 60))
					else:
						longitude = ''

					altitude = gpsSplit[9]

					if (len(altitude) > 0):
						try:
							self.processAltitude(float(altitude))
						except:
							print("Unable to cast Altitude to a float")

					formattedGpsString = "{},{},{},{}".format(time, latitude, longitude, altitude)
				else:
					print('line not correct: ' + str(gpsSplit))
			except:
				print('Format data: no valid GPS string - exception caught')
				print('offending string: ' + str(gpsString))
				formattedGpsString = ",,,"
				validGpsData = False

		return formattedGpsString, validGpsData

	def gpsSerialInput(self):
		messageReceived = "NO_GPS_DATA\n"
		serialInput = ""
		retries = 5
		iterationsToWait = 10

		try:
			while (retries > 0 and iterationsToWait > 0):
				if (self.gpsSerialPort.inWaiting() > 0):  # If there's a buffer for us to read
					serialInput = self.gpsSerialPort.readline()
					if (serialInput[:6] == r"$GPGGA"):  # Makes sure this is the line we want
						break  # This is our stop
					else:
						# print("Discarding unused data: " + serialInput)
						logGpsData(serialInput)
						serialInput = ""  # This is not the data we're looking for
						retries -= 1
				else:
					print('GPS serial port is not waiting')
					time.sleep(0.1)
					iterationsToWait -= 1

		except:
			print("Exception thrown while trying to read GPS serial input")

		if (retries > 0 and iterationsToWait > 0):  # We found what we wanted
			messageReceived = serialInput

		return messageReceived


	def radioSerialInput(self):
		serialInput = ""

		try:
			if not (self.radioSerialPort.inWaiting()):
				time.sleep(0.5)
			while(self.radioSerialPort.inWaiting()):
				serialInput += self.radioSerialPort.readline()
			if (len(serialInput) > 1):
				print("Serial input from radio: " + serialInput)
			else:
				print("No serial input to be read from the radio")
		except:
			print("Unable to read from radio serial port.")

		return serialInput


	def sendSerialOutput(self, line):
		try:
			print(RADIO_CALLSIGN + "," + line + ",END_TX\n")
			line = self.radioSerialPort.write(RADIO_CALLSIGN + "," + line + ",END_TX\n")
		except:
			print("Unable to write to radio serial port on " + RADIO_SERIAL_PORT)

	def openRadioSerialPort(self):
		try:
			if (self.radioSerialPort.isOpen()):
				self.radioSerialPort.close()
		except:
			print("Failed to close the radio serial port " + RADIO_SERIAL_PORT + ". Was it ever opened?")

		try:
			self.radioSerialPort = serial.Serial(port = RADIO_SERIAL_PORT, baudrate = RADIO_BAUDRATE, timeout = 1)
		except:
			print("Failed to open the radio GPS serial port")

	def openGpsSerialPort(self):
		try:
			if (self.gpsSerialPort.isOpen()):
				self.gpsSerialPort.close()
		except:
			print("GPS serial port " + GPS_SERIAL_PORT + " won't close. May not have been open")

		try:
			self.gpsSerialPort = serial.Serial(port = GPS_SERIAL_PORT, baudrate = GPS_BAUDRATE, timeout = 1)
		except:
			print("Failed to open the GPS serial port")

	def releaseBalloon(self):
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(11, GPIO.OUT)

	 	GPIO.output(11, 0)

	 	while True:
	 		# if str(password) == 'SSAGhabRELEASE':
	 			zero = time.time()
	 			while time.time() - zero < 300:
	 				try:
	 					ret, frame = cap.read()
	 					out.write(frame)
	 					cv2.waitKey(1)
	 				except:
	 					pass
	 				GPIO.output(11, 1)
	 			try:
	 				out.release()
	 				cap.release()
	 			except:
	 				pass
	 			break
	 		# else:
	 			# password = raw_input()
	 	GPIO.cleanup()

		'''
		NOTES
		
		Orange wire must be connected to pin 5 of MOSFET
		Yellow wire must be connected to pin 7 of MOSFET
		If raspi pin is high, motor will retract
		If raspi pin is low, motor will expand
		
		'''
	def brmReset(self):
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(11, GPIO.OUT)

	 	GPIO.output(11, 0)

	def snapShot(self):
		if str(snapCommand) == 'snapshot':
			ret, frame = cap.read()
			cv2.imwrite('snapshot' + str(snapCount) + '.png', frame)
			snapCount = snapCount + 1
		else:
			snapCommand = raw_input()

def logRadioMessage(line):
	try:
		radioLogFile = open(RADIO_LOG_FILE_LOCATION, "a")
	except:
		radioLogFile = open(RADIO_LOG_FILE_LOCATION, "w")

	radioLogFile.write(str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ": " + line + "\n")
	radioLogFile.close

def logScript(line):
	try:
		scriptLogFile = open(SCRIPT_LOG_FILE_LOCATION, "a")
	except:
		scriptLogFile = open(SCRIPT_LOG_FILE_LOCATION, "w")

	scriptLogFile.write(str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ": " + line + "\n")
	scriptLogFile.close

def logGpsData(line):
	try:
		gpsLogFile = open(GPS_LOG_FILE_LOCATION, "a")
	except:
		gpsLogFile = open(GPS_LOG_FILE_LOCATION, "w")

	gpsLogFile.write(str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ": " + line + "\n")
	gpsLogFile.close

if __name__ == '__main__':
	runScript = balloonScript()
