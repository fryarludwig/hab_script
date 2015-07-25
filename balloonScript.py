from __future__ import print_function


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
import signal
import math
import smbus
import subprocess
import os
import sys
import numpy
import RPi.GPIO as GPIO
import time
import cv2
import datetime

RADIO_CALLSIGN = "hab"
RADIO_BAUDRATE = 38400

GPS_BAUDRATE = 4800

GPS_LOG_FILE_LOCATION = r"/home/pi/hab_script/logData/gps_log.txt"
RADIO_LOG_FILE_LOCATION = r"/home/pi/hab_script/logData/radio_log.txt"
SCRIPT_LOG_FILE_LOCATION = r"/home/pi/hab_script/logData/balloon_script_log.txt"

TIME_TO_DRIVE_BRM = 15

RELEASE_BALLOON_ALTITUDE = 21945  # 72,000 feet

EXCEPTION_SUM = 0
exceptionList = {'RADIO_TRANSMIT': 'Exception while transmitting through radio',
		 'RADIO_RECEIVE': 'Exception while receiving radio packet',
		 'GPS_RECEIVE': 'Exception while receiving GPS packet',
		 'GPS_HANDLING': 'Exception while handling GPS packet',
		 'TEMP_RPI': 'Exception while reading RPi temperature',
		 'TEMP_EXT': 'Exception while reading external temperature',
		 'TEMP_BAT': 'Exception while reading battery temperature',
		 'VOLT_BAT': 'Exception while reading battery voltage',
		 'RH': 'Exception while reading humidity sensor',
		 'ACCEL': 'Exception while reading accelerometer',
		 'MESSAGE_HANDLING': 'Exception while processing line',
		 'VIDEO_RECORD': 'Exception while initiating video',
		 'BALLOON_RELEASE': 'Exception while releasing balloon',
		 'BRM_RESET': 'Exception while resetting BRM',
		 'USB_SWITCH': 'Exception while switching radio and GPS USB ports',
		 'SNAPSHOT': 'Exception while taking snapshot burst',
		 'MAIN_SCRIPT': 'Exception while running script',
		 'UNKNOWN': 'Unknown exception occured'
		 }


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

moment = [None, None, None]
VALID_CAMERA = True

class balloonScript():
	def __init__(self):
		self.foundCorrectUSB = False
		self.RADIO_SERIAL_PORT = "/dev/ttyUSB0"
		self.GPS_SERIAL_PORT = "/dev/ttyUSB1"

		self.snapCount = 1
		self.burstCount = 1
		self.videoCount = 1

		self.brmRecorded = False
		self.videoEndTime = time.mktime(datetime.datetime.now().timetuple())

		# Open output files

		self.scriptLog = open(SCRIPT_LOG_FILE_LOCATION, "a")
		self.gpsLog = open(GPS_LOG_FILE_LOCATION, "a")
		self.fTrpi = open('/home/pi/hab_script/sensorData/temp_raspi.txt', 'a')
		self.fText = open('/home/pi/hab_script/sensorData/temp_external.txt', 'a')
		self.fTbat = open('/home/pi/hab_script/sensorData/temp_batteries.txt', 'a')
		self.fVbat = open('/home/pi/hab_script/sensorData/voltage_batteries.txt', 'a')
		self.fRH = open('/home/pi/hab_script/sensorData/humidity.txt', 'a')
		self.fAcc = open('/home/pi/hab_script/sensorData/accelerometer.txt', 'a')

		# Set up variables to be used by the script
		self.altitudeDataList = []
		self.balloonReleaseArmed = False
		self.balloonReleaseActivated = False

		# Open serial ports
		self.radioSerialPort = None
		self.gpsSerialPort = None

		try:
			self.openRadioSerialPort()
			self.openGpsSerialPort()
		except:
			pass

		print('starting balloon script')

		self.runBalloonScript()

	def runBalloonScript(self):
		self.sendSerialOutput("warn,STARTING_SCRIPT")

		while(True):
	 		try:
				time.sleep(1)
				gpsMessage, validGpsData = self.processGpsData(self.gpsSerialInput())
				sensorData, validSensorData = self.getSensorData()

				if (validGpsData or validSensorData):
					self.sendSerialOutput("data," + gpsMessage + sensorData + str(EXCEPTION_SUM))

				EXCPETION_SUM = 0
	 		except:
				print("Main loop handled uncaught exception. Exiting now.")
				sys.exit()

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
						self.foundCorrectUSB = True
						if (line[4:7] == "cmd"):
							self.processCommand(line[8:])

					elif (line[:6] == "chase1" or
						line[:6] == "chase2" or
						line[:6] == "chase3"):
						self.foundCorrectUSB = True
						if (line[7:10] == "cmd"):
							self.processCommand(line[11:])

			logRadioMessage(str(message))

		except:
			logRadioMessage("Data caused exception: " + str(message))
			print("Exception in handling received message")
			exceptionDictionary('MESSAGE_HANDLING', 10)

	def processCommand(self, command):
		if (command == "ARM_BRM"):
			self.balloonReleaseArmed = True
			self.sendSerialOutput("ack,BRM_ARMED")
			print("ARMING BALLOON")
		elif (command == "SNAPSHOT"):
			print("SNAPSHOT " + str(self.burstCount) + " TAKEN")
			self.snapShot()
			self.sendSerialOutput("ack,SNAPSHOT_TAKEN")
		elif (command == "DISK_SPACE"):
			self.sendSerialOutput("ack," + self.getDiskSpace())
		elif (command == "RESET_BRM"):
			self.balloonReleaseArmed = False
			self.brmReset()
			self.sendSerialOutput("ack,BRM_RESET")
			print("BALLOON RELEASE RESET")
		elif (command == "DISARM_BRM"):
			self.balloonReleaseArmed = False
			self.sendSerialOutput("ack,BRM_DISARMED")
			print("DISARMING BALLOON")
		elif (command == "SSAG_RELEASE_BALLOON" and
				self.balloonReleaseArmed):
			self.releaseBalloon()
			print("BALLOON RELEASE ACTIVATED")
			self.sendSerialOutput("ack,BRM_ACTIVATED")
		else:
			print("COMMAND NOT RECOGNIZED")
			self.sendSerialOutput("ack,NO_OP")

	def getSensorData(self):
		validSensorData = 6

		calculatedPiTemp = "NULL"
		calculatedExternalTemp = "NULL"
		calculatedBatteryTemp = "NULL"
		calculatedVoltageBattery = "NULL"
		calculatedRHValue = "NULL"
		rawAccelX = "NULL"
		rawAccelY = "NULL"
		rawAccelZ = "NULL"

		try:
			rawValPiTemp = bus.read_byte_data(address, registerTrpi)  # 4604
			VTrpi = rawValPiTemp * Vin / maxAD
			Rrpi = 51800 * ((Vin / VTrpi) - 1)
			PiTempRHPurpose = (1 / ((.0014782389) + (.00023632193 * (math.log(Rrpi))) + ((.00000011403386 * (math.log(Rrpi))) ** 3)) - 273.15)
			calculatedPiTemp = "%4.1f" % (1 / ((.0014782389) + (.00023632193 * (math.log(Rrpi))) + ((.00000011403386 * (math.log(Rrpi))) ** 3)) - 273.15)
			self.fTrpi = open('/home/pi/hab_script/sensorData/temp_raspi.txt', 'a')
			self.fTrpi.write(str(VTrpi) + ' ' + calculatedPiTemp + '\n')
			self.fTrpi.close()
		except:
			validSensorData -= 1
			exceptionDictionary('TEMP_RPI', 4)

		try:
			rawValExternalTemp = bus.read_byte_data(address, registerText)  # 4599
			VText = rawValExternalTemp * 3.3 / 255
			Rext = 51800 * ((Vin / VText) - 1)
			calculatedExternalTemp = "%4.1f" % (1 / ((.0014732609) + (.00023727640 * (math.log(Rext))) + ((.00000010814580 * (math.log(Rext))) ** 3)) - 273.15)
			self.fText = open('/home/pi/hab_script/sensorData/temp_external.txt', 'a')
			self.fText.write(str(VText) + ' ' + calculatedExternalTemp + '\n')
			self.fText.close()
		except:
			validSensorData -= 1
			exceptionDictionary('TEMP_EXT', 5)

		try:
			rawValBatteryTemp = bus.read_byte_data(address, registerTbat)  # 4600
			VTbat = rawValBatteryTemp * 3.3 / 255
			Rbat = 51800 * ((Vin / VTbat) - 1)
			calculatedBatteryTemp = "%4.1f" % ((1 / ((.0014721232) + (.00023728796 * (math.log(Rbat))) + ((.00000010792173 * (math.log(Rbat))) ** 3)) - 273.15))
			self.fTbat = open('/home/pi/hab_script/sensorData/temp_batteries.txt', 'a')
			self.fTbat.write(str(VTbat) + ' ' + calculatedBatteryTemp + '\n')
			self.fTbat.close()
		except:
			validSensorData -= 1
			exceptionDictionary('TEMP_BAT', 6)

		try:
			rawValBatteryVoltage = bus.read_byte_data(address, registerVbat)
			calculatedVoltageBattery = "%4.1f" % (((10.08 + 30.05) / 10.08) * rawValBatteryVoltage * 3.3 / 255)
			self.fVbat = open('/home/pi/hab_script/sensorData/voltage_batteries.txt', 'a')
			self.fVbat.write(str(rawValBatteryVoltage) + ' ' + calculatedVoltageBattery + '\n')
			self.fVbat.close()
		except:
			validSensorData -= 1
			exceptionDictionary('VOLT_BAT', 7)

		try:
			rawValRH = bus.read_byte_data(address, registerRH)
			VRH = rawValRH * 3.3 * ((10.1 + 38.5) / 38.5) / 255
			RHApprox = ((VRH / Vsupply) - .16) / .0062
			calculatedRHValue = "%4.1f" % ((RHApprox) / (1.0546 - (.00216 * PiTempRHPurpose)))
			self.fRH = open('/home/pi/hab_script/sensorData/humidity.txt', 'a')
			self.fRH.write(str(rawValRH) + ' ' + calculatedRHValue + '\n')
			self.fRH.close()
		except:
			validSensorData -= 1
			exceptionDictionary('RH', 8)

		try:
			i = 0
			rawAccelX = str(bus.read_byte_data(address, registerAccX))
			rawAccelY = str(bus.read_byte_data(address, registerAccY))
			rawAccelZ = str(bus.read_byte_data(address, registerAccZ))
			positionZero = [128, 128, 152]
			rawMoment = [rawAccelX, rawAccelY, rawAccelZ]
			while i < 3:
				moment[i] = abs(rawMoment[i] - positionZero[i])
				i = i + 1
			calculatedMagnitude = round(((moment[0]) ** 2 + (moment[1]) ** 2 + (moment[2]) ** 2) ** (.5), 2)
			self.fAcc = open('/home/pi/hab_script/sensorData/accelerometer.txt', 'a')
			self.fAcc.write(str(moment[0]) + ' ' + str(moment[1]) + ' ' + str(moment[2]) + ' ' + str(calculatedMagnitude) + '\n')
			self.fAcc.close()
		except:
			validSensorData -= 1
			exceptionDictionary('ACCEL', 9)


		return ",{},{},{},{},{},{},{},{}".format(calculatedPiTemp, calculatedExternalTemp,
							calculatedBatteryTemp, calculatedVoltageBattery,
							calculatedRHValue, rawAccelX, rawAccelY, rawAccelZ), (validSensorData > 0)

	def processAltitude(self, currAlt):
		try:
			if (currAlt > RELEASE_BALLOON_ALTITUDE):
				numOfDataPoints = len(self.altitudeDataList)

				self.altitudeDataList.append(currAlt)

				if (numOfDataPoints > 3):  # We need 3 consecutive data points
					logScript("3 heights above target: " + str(self.altitudeDataList))
					self.releaseBalloon()
			elif currAlt > (RELEASE_BALLOON_ALTITUDE - 500):
				if self.brmRecorded == False:
					self.recordVideo(20)
					self.brmRecorded = True
					print('recording balloon release')
					self.sendSerialOutput('ack,BRM_RECORDING_STARTED')

			else:
				self.altitudeDataList.clear()

		except:
			print("Error in calculating altitude change")


	def processGpsData(self, gpsString):
		validGpsData = False
		formattedGpsString = 'NULL,NULL,NULL,NULL'

		if (gpsString != "NO_GPS_DATA\n" and len(gpsString) > 0):
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
					try:
						self.processAltitude(float(altitude))
					except:
						pass

					if (len(altitude) > 0):
						try:
							self.processAltitude(float(altitude))
						except:
							print("Unable to cast Altitude to a float")

					formattedGpsString = "{},{},{},{}".format(time, latitude, longitude, altitude)
				else:
					logGpsData('Line length unexpected - incomplete packet: ' + str(gpsSplit))
					print('Line length unexpected - incomplete packet: ' + str(gpsSplit))
			except:
				print('Format data: no valid GPS string - exception caught')
				exceptionDictionary('LINE_HANDLING', 10)
				print('offending string: ' + str(gpsString))
				formattedGpsString = ",,,"
				validGpsData = False

		return formattedGpsString, validGpsData

	def gpsSerialInput(self):
		messageReceived = "NO_GPS_DATA\n"
		serialInput = ""
		retries = 10
		iterationsToWait = 15

		if (self.foundCorrectUSB):
			self.gpsSerialPort.flushOutput()
			self.gpsSerialPort.flushInput()
			time.sleep(.2)

		try:
			while (retries > 0 and iterationsToWait > 0):
				if (self.gpsSerialPort.inWaiting() > 0):  # If there's a buffer for us to read
					serialInput = self.gpsSerialPort.readline()
					if (serialInput[:6] == r"$GPGGA"):  # Makes sure this is the line we want
						self.foundCorrectUSB = True
						break  # This is our stop
					else:
						# print("Discarding unused data: " + serialInput)
						logGpsData(serialInput)
						serialInput = ""  # This is not the data we're looking for
						retries -= 1
				else:
					print('GPS serial port is not ready')
					time.sleep(0.1)
					iterationsToWait -= 1
		except:
			print("Exception thrown while trying to read GPS serial input")
			exceptionDictionary('GPS_RECEIVE', 3)
			if not (self.foundCorrectUSB):
				self.switchUSB()  # REVIEW THIS LATER

		if (retries > 0 and iterationsToWait > 0):  # We found what we wanted
			messageReceived = serialInput

		elif not (self.foundCorrectUSB):
			self.switchUSB()

		logGpsData(messageReceived)
		print(messageReceived)
		return messageReceived


	def radioSerialInput(self):
		serialInput = ""
		counter = 10

		try:
			if not (self.radioSerialPort.inWaiting()):
				time.sleep(0.2)
			while(self.radioSerialPort.inWaiting() and counter > 0):
				serialInput += self.radioSerialPort.readline(1024)
				counter -= 1
			if (len(serialInput) > 1):
				print("Serial input from radio: " + serialInput)
			else:
				print("No serial input to be read from the radio")
		except:
			print("Unable to read from radio serial port.")
			exceptionDictionary('RADIO_RECEIVE', 2)

		return serialInput


	def sendSerialOutput(self, line):
		try:
			self.radioSerialPort.write(RADIO_CALLSIGN + "," + str(line) + ",END_TX\n")
		except:
			print("Unable to write to radio serial port on " + self.RADIO_SERIAL_PORT)
			exceptionDictionary('RADIO_TRANSMIT', 1)

	def openRadioSerialPort(self):
		try:
			if (self.radioSerialPort.isOpen()):
				self.radioSerialPort.close()
		except:
			print("Failed to close the radio serial port " + self.RADIO_SERIAL_PORT + ". Was it ever opened?")

		try:
			self.radioSerialPort = serial.Serial(port = self.RADIO_SERIAL_PORT, baudrate = RADIO_BAUDRATE, timeout = 1)
			print('Opening Radio port ' + self.RADIO_SERIAL_PORT + ' at baud ' + str(RADIO_BAUDRATE))
		except:
			print("Failed to open the radio GPS serial port")

	def openGpsSerialPort(self):
		try:
			if (self.gpsSerialPort.isOpen()):
				self.gpsSerialPort.close()
		except:
			print("GPS serial port " + self.GPS_SERIAL_PORT + " won't close. May not have been open")

		try:
			self.gpsSerialPort = serial.Serial(port = self.GPS_SERIAL_PORT, baudrate = GPS_BAUDRATE, timeout = 1)
			print('Opened GPS port ' + self.GPS_SERIAL_PORT + ' at baud ' + str(GPS_BAUDRATE))
		except:
			print("Failed to open the GPS serial port")

	def releaseBalloon(self):
		try:
			GPIO.setmode(GPIO.BCM)
			GPIO.setwarnings(False)
			GPIO.setup(23, GPIO.OUT)

			i = 0

			if self.brmRecorded == False:
				try:
					self.recordVideo(20)

					self.brmRecorded = True

					self.sendSerialOutput('ack,BRM_RECORDING_STARTED')

					GPIO.output(23, 1)
				except:
					print("Unable to start recording!")
					exceptionDictionary('VIDEO_RECORD', 11)

			GPIO.output(23, 1)

			try:
				self.snapShot()
				self.snapShot()
			except:
				print ("Snapshots failed to take")

			self.balloonReleaseActivated = True

		except:
			logScript("Unable to release the balloon. Unknown exception occurred")
			self.radioSerialOutput("error,release_balloon_cmd_failed")
			exceptionDictionary('BALLOON_RELEASE', 12)


	def brmReset(self):
		try:
			GPIO.setmode(GPIO.BCM)
			GPIO.setup(23, GPIO.OUT)
			GPIO.setwarnings(False)

			GPIO.output(23, 0)

			self.balloonReleaseActivated = False

		except:
			exceptionDictionary('BRM_RESET', 13)



	def snapShot(self, numberOfPhotosToBurst = 5):
		try:
			currTimeInSeconds = time.mktime(datetime.datetime.now().timetuple())
			timeLeftToRecord = self.videoEndTime - currTimeInSeconds

			photosTaken = 0

			print("Checking if camera is available")
			print("Video time left: " + str(timeLeftToRecord))

			if ((timeLeftToRecord) >= 0 and (timeLeftToRecord) < 5):
				sleep(timeLeftToRecord + 1)

			if ((timeLeftToRecord) < 0):
				print("Attempting to initialize camera")
				cap = cv2.VideoCapture(0)
				time.sleep(2)
				print("Opened camera!")

				while os.path.isfile('/home/pi/hab_script/snapshots/burst' + str(self.burstCount) + 'snapshot0.png'):
					self.burstCount = self.burstCount + 1

				while photosTaken <= numberOfPhotosToBurst:
					ret, frame = cap.read()
					cv2.imwrite('/home/pi/hab_script/snapshots/burst' + str(self.burstCount) + 'snapshot' + str(photosTaken) + '.png', frame)
					photosTaken = photosTaken + 1
					print('Taking photo')

				cap.release
			else:
				print("Cannot record images - still recording video for the next " + str(timeLeftToRecord) + ' seconds')

		except:
			print('taking pictures, encountered exception')
			exceptionDictionary('SNAPSHOT', 0)

	def recordVideo(self, duration):

		currTimeInSeconds = time.mktime(datetime.datetime.now().timetuple())
		timeLeftToRecord = self.videoEndTime - currTimeInSeconds

		print("Video time left: " + str(timeLeftToRecord))
		if ((timeLeftToRecord) >= 0 and (timeLeftToRecord) < 5):
			sleep(timeLeftToRecord + 1)

		if ((timeLeftToRecord) < 0):
			while os.path.isfile('/home/pi/hab_script/videos/video' + str(self.videoCount) + '.avi'):
				self.videoCount += 1

			videoRecordingString = "sudo avconv -an -f video4linux2 -s 560x480  -r 15 -i /dev/video0 -timelimit "
			videoRecordingString += (str(duration) + " /home/pi/hab_script/videos/video" + str(self.videoCount) + ".avi")
			subprocess.Popen(videoRecordingString, shell = True)

			self.videoEndTime = (currTimeInSeconds + duration + 10)

			self.videoCount += 1

		else:
			print("Cannot record video - still recording video for the next " + str(timeLeftToRecord) + ' seconds')

	def switchUSB(self):
		try:
			if self.RADIO_SERIAL_PORT == "/dev/ttyUSB1":
				self.RADIO_SERIAL_PORT = "/dev/ttyUSB0"
				self.GPS_SERIAL_PORT = "/dev/ttyUSB1"
			else:
				self.RADIO_SERIAL_PORT = "/dev/ttyUSB1"
				self.GPS_SERIAL_PORT = "/dev/ttyUSB0"

			print('Switching USB ports')
			self.radioSerialPort.close()
			self.gpsSerialPort.close()

			time.sleep(2)

			self.openRadioSerialPort()
			self.openGpsSerialPort()
		except:
			exceptionDictionary('SWITCH_USB', 14)

	def getDiskSpace(self):
		output = subprocess.check_output(['df', '-h'])
		return str(output.split()[10])


def logRadioMessage(line):
	radioLogFile = open(RADIO_LOG_FILE_LOCATION, "a")

	radioLogFile.write(str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ": " + line + "\n")
	radioLogFile.close

def logScript(line):
	scriptLogFile = open(SCRIPT_LOG_FILE_LOCATION, "a")

	scriptLogFile.write(str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ": " + line + "\n")
	scriptLogFile.close

def logGpsData(line):
	gpsLogFile = open(GPS_LOG_FILE_LOCATION, "a")

	gpsLogFile.write(str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ": " + line + "\n")
	gpsLogFile.close

def logError(line):
	exceptionLogFile = open('/home/pi/hab_script/logData/exception_log.txt', "a")

	exceptionLogFile.write(str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ": " + line + "\n")
	exceptionLogFile.close


def errorDictionary(error, message):
	if error == "SNAPSHOT":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 0)
		logError(message)
	elif error == "RADIO_TRANSMIT":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 1)
		logError(message)
	elif error == "RADIO_RECEIVE":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 2)
		logError(message)
	elif error == "GPS_RECEIVE":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 3)
		logError(message)
	elif error == "TEMP_RPI":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 4)
		logError(message)
	elif error == "TEMP_EXT":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 5)
		logError(message)
	elif error == "TEMP_BAT":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 6)
		logError(message)
	elif error == "VOLT_BAT":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 7)
		logError(message)
	elif error == "RH":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 8)
		logError(message)
	elif error == "ACCEL":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 9)
		logError(message)
	elif error == "MESSAGE_HANDLING":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 10)
		logError(message)
	elif error == "VIDEO_RECORD":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 11)
		logError(message)
	elif error == "BALLOON_RELEASE":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 12)
		logError(message)
	elif error == "BRM_RESET":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 13)
		logError(message)
	elif error == "USB_SWITCH":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 14)
		logError(message)
	elif error == "UNKNOWN":
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << 15)
		logError(message)
	else:
		logError('Error while receiving error')

def exceptionDictionary(error, bitShifter):
	try:
		logError(exceptionList[str(error)])
		EXCEPTION_SUM = EXCEPTION_SUM + (1 << int(bitShifter))
	except:
		logError('Exception while attempting to find exception')

if __name__ == '__main__':
	while(True):
		try:
			runScript = balloonScript()
		except:
			errorDictionary("MAIN_SCRIPT", "Program terminated, starting again")
			print("\n\nProgram terminated, starting again\n\n")
			sys.exit()
			time.sleep(5)
