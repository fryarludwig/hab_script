from __future__ import print_function


"""

Misc. Notes:

- If i2c directory is not found:
	- Enable through raspi-config
	- Edit /etc/modules and add these two lines: "i2c-bcm2708" and "i2c-dev"
	- Reboot
- GPS via UART freezes - unsure as to the reason why
	- Currently running GPS via USB, and this works flawlessly

TODO:


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

logFileLocationDictionary = {"STARTUP" : r"/home/pi/hab_script/logData/startcount_log.txt",
							"GPS" : r"/home/pi/hab_script/logData/gps_log.txt",
							"RADIO" : r"/home/pi/hab_script/logData/radio_log.txt",
							"SCRIPT" : r"/home/pi/hab_script/logData/balloon_script_log.txt",
							"EXCEPTION" : r'/home/pi/hab_script/logData/exception_log.txt',
							"RPI_TEMP" : r'/home/pi/hab_script/sensorData/temp_raspi.txt',
							"EXT_TEMP" : r'/home/pi/hab_script/sensorData/temp_external.txt',
							"BAT_TEMP" : r'/home/pi/hab_script/sensorData/temp_batteries.txt',
							"VOLTAGE" : r'/home/pi/hab_script/sensorData/voltage_batteries.txt',
							"HUMIDITY" : r'/home/pi/hab_script/sensorData/humidity.txt',
							"ACCEL" : r'/home/pi/hab_script/sensorData/accelerometer.txt'}

TIME_TO_DRIVE_BRM = 15

RELEASE_BALLOON_ALTITUDE = 21945  # 72,000 feet

exceptionDictionary = {'RADIO_TRANSMIT': 'Exception while transmitting through radio',  # 1
		 'RADIO_RECEIVE': 'Exception while receiving radio packet',  # 2
		 'GPS_RECEIVE': 'Exception while receiving GPS packet',  # 3
		 'GPS_HANDLING': 'Exception while handling GPS packet',  # 16
		 'TEMP_RPI': 'Exception while reading RPi temperature',  # 4
		 'TEMP_EXT': 'Exception while reading external temperature',  # 5
		 'TEMP_BAT': 'Exception while reading battery temperature',  # 6
		 'VOLT_BAT': 'Exception while reading battery voltage',  # 7
		 'RH': 'Exception while reading humidity sensor',  # 8
		 'ACCEL': 'Exception while reading accelerometer',  # 9
		 'MESSAGE_HANDLING': 'Exception while processing line',  # 10
		 'VIDEO_RECORD': 'Exception while initiating video',  # 11
		 'BALLOON_RELEASE': 'Exception while releasing balloon',  # 12
		 'BRM_RESET': 'Exception while resetting BRM',  # 13
		 'USB_SWITCH': 'Exception while switching radio and GPS USB ports',  # 14
		 'SNAPSHOT': 'Exception while taking snapshot burst',  # 0
		 'MAIN_SCRIPT': 'Exception while running script, restarting',  # 15
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
	def __init__(self, EXCEPTION_SUM = 0):
		self.sentStartupNumber = False

		self.foundCorrectUSB = False
		self.RADIO_SERIAL_PORT = "/dev/ttyUSB0"
		self.GPS_SERIAL_PORT = "/dev/ttyUSB1"

		self.snapCount = 1
		self.burstCount = 1
		self.videoCount = 1
		self.lastBurstTime = time.time()
		self.numberOfPhotosToBurst = 5
		self.intervalDuration = 30
		self.intervalCount = 0
		self.snapshotIntervalEnabled = True

		self.EXCEPTION_SUM = EXCEPTION_SUM

		self.brmRecorded = False
		self.videoEndTime = time.mktime(datetime.datetime.now().timetuple())

		# Set up variables to be used by the script
		self.altitudeDataList = []
		self.balloonReleaseArmed = False
		self.balloonReleaseActivated = False

		# Open serial ports
		self.radioSerialPort = None
		self.gpsSerialPort = None

		self.openRadioSerialPort()
		self.openGpsSerialPort()

		print('starting balloon script')

		self.runBalloonScript()

	def runBalloonScript(self):

		fstartup = open(logFileLocationDictionary["STARTUP"], 'r')
		print('1')

		for line in fstartup:
			try:
				startupLine = line.split(" ")
				startupNumber = int(startupLine[2])
			except:
				startupLine = 0
				startupNumber = 0

		log("STARTUP", str(startupNumber + 1))

		while(True):

			gpsMessage, validGpsData = self.processGpsData(self.gpsSerialInput())
			sensorData, validSensorData = self.getSensorData()

			if (validGpsData or validSensorData):

				print('Sending serial out')
				self.sendSerialOutput("data," + gpsMessage + sensorData + ',' + str(self.EXCEPTION_SUM))

			self.EXCEPTION_SUM = 0

			# receive
			messageReceived = self.radioSerialInput()

			# act on receive
			self.handleMessage(messageReceived)

			log('RADIO', messageReceived)

			if self.foundCorrectUSB and not self.sentStartupNumber:
				self.sendSerialOutput("init,STARTING_SCRIPT," + str(startupNumber + 1))
				self.sentStartupNumber = True

			if (self.snapshotIntervalEnabled):
				if (time.time() - self.lastBurstTime) >= self.intervalDuration:
					self.snapShot(self.numberOfPhotosToBurst)
					self.lastBurstTime = time.time()

	def handleMessage(self, message):
		try:
			for line in message.split(',END_TX\n'):
				if (len(line) > 0):
					if (line[:3] == "nps"):
						self.foundCorrectUSB = True
						if (line[4:7] == "cmd"):
							self.processCommand(line[8:])
						self.foundCorrectUSB = True

					elif (line[:6] == "chase1" or
						line[:6] == "chase2" or
						line[:6] == "chase3"):
						self.foundCorrectUSB = True
						if (line[7:10] == "cmd"):
							self.processCommand(line[11:])
						self.foundCorrectUSB = True

				log("RADIO", str(message))

		except:
			print("Exception in handling received message")
			log("RADIO", ("Data caused exception: " + str(message)))
			self.exceptionDictionary('MESSAGE_HANDLING', 10)

	def processCommand(self, command):
		try:
			if (len(command) > 0):
				if (command == "ARM_BRM"):
					self.balloonReleaseArmed = True
					self.sendSerialOutput("ack,BRM_ARMED")
					print("ARMING BALLOON")

				elif (command[:8] == "SNAPSHOT"):
					commandSplit = command.split(',')
					try:
						if (int(commandSplit[2]) == -1):
							self.snapShot(int(commandSplit[1]))
						elif (int(commandSplit[2]) == 0):
							self.snapshotIntervalEnabled = False
						else:
							self.snapshotIntervalEnabled = True
							self.numberOfPhotosToBurst = int(commandSplit[1])
							self.intervalDuration = int(commandSplit[2])

						self.sendSerialOutput('ack,SNAPSHOT_UPDATE,' + str(commandSplit[1]) + ',' + str(commandSplit[2]))
					except:
						log('RADIO', 'Unable to parse snapshot command')

				elif (command == "DISK_SPACE"):
					self.sendSerialOutput("ack,DISK," + self.getDiskSpace())

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

		except:
			self.exceptionDictionary("MESSAGE_HANDLING", 10)

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
			log("RPI_TEMP", str(VTrpi) + ' ' + calculatedPiTemp)
		except:
			validSensorData -= 1
			self.exceptionDictionary('TEMP_RPI', 4)

		try:
			rawValExternalTemp = bus.read_byte_data(address, registerText)  # 4599
			VText = rawValExternalTemp * 3.3 / 255
			Rext = 51800 * ((Vin / VText) - 1)
			calculatedExternalTemp = "%4.1f" % (1 / ((.0014732609) + (.00023727640 * (math.log(Rext))) + ((.00000010814580 * (math.log(Rext))) ** 3)) - 273.15)
			log("EXT_TEMP", str(VText) + ' ' + calculatedExternalTemp)
		except:
			validSensorData -= 1
			self.exceptionDictionary('TEMP_EXT', 5)

		try:
			rawValBatteryTemp = bus.read_byte_data(address, registerTbat)  # 4600
			VTbat = rawValBatteryTemp * 3.3 / 255
			Rbat = 51800 * ((Vin / VTbat) - 1)
			calculatedBatteryTemp = "%4.1f" % ((1 / ((.0014721232) + (.00023728796 * (math.log(Rbat))) + ((.00000010792173 * (math.log(Rbat))) ** 3)) - 273.15))
			log("BAT_TEMP", str(VTbat) + ' ' + calculatedBatteryTemp)
		except:
			validSensorData -= 1
			self.exceptionDictionary('TEMP_BAT', 6)

		try:
			rawValBatteryVoltage = bus.read_byte_data(address, registerVbat)
			calculatedVoltageBattery = "%4.1f" % (((10.08 + 30.05) / 10.08) * rawValBatteryVoltage * 3.3 / 255)
			log("VOLTAGE", str(rawValBatteryVoltage) + ' ' + calculatedVoltageBattery)
		except:
			validSensorData -= 1
			self.exceptionDictionary('VOLT_BAT', 7)

		try:
			rawValRH = bus.read_byte_data(address, registerRH)
			VRH = rawValRH * 3.3 * ((10.1 + 38.5) / 38.5) / 255
			RHApprox = ((VRH / Vsupply) - .16) / .0062
			calculatedRHValue = "%4.1f" % ((RHApprox) / (1.0546 - (.00216 * PiTempRHPurpose)))
			log("HUMIDITY", str(rawValRH) + ' ' + calculatedRHValue)
		except:
			validSensorData -= 1
			self.exceptionDictionary('RH', 8)

		try:
			rawAccelX = str(bus.read_byte_data(address, registerAccX))
			rawAccelY = str(bus.read_byte_data(address, registerAccY))
			rawAccelZ = str(bus.read_byte_data(address, registerAccZ))
			log("ACCEL", str(rawAccelX) + ' ' + str(rawAccelY) + ' ' + str(rawAccelZ))
		except:
			validSensorData -= 1
			self.exceptionDictionary('ACCEL', 9)

		return ",{},{},{},{},{},{},{},{}".format(calculatedPiTemp, calculatedExternalTemp,
							calculatedBatteryTemp, calculatedVoltageBattery,
							calculatedRHValue, rawAccelX, rawAccelY, rawAccelZ), (validSensorData > 0)

	def processAltitude(self, currAlt):
		try:
			if (currAlt > RELEASE_BALLOON_ALTITUDE):
				numOfDataPoints = len(self.altitudeDataList)
				print("CurrAlt is above release altitude")

				self.altitudeDataList.append(currAlt)

				if (numOfDataPoints > 3):  # We need 3 consecutive data points
					log("SCRIPT", "3 heights above target: " + str(self.altitudeDataList))
					self.releaseBalloon()

			elif currAlt > (RELEASE_BALLOON_ALTITUDE - 500):
				print("CurrAlt is getting close to the release altitude")
				if not (self.brmRecorded):
					self.recordVideo(240)
					self.brmRecorded = True
					print('recording balloon release')
					self.sendSerialOutput('ack,BRM_RECORDING_STARTED')

			else:
				print("CurrAlt isn't close to release altitude")
				self.altitudeDataList = []

		except:
			print("Error in calculating altitude change")


	def processGpsData(self, gpsString):
		try:
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
						if (len(altitude) > 0):
							try:
								floatAltitude = float(altitude)
								self.processAltitude(floatAltitude)
							except:
								print("Unable to cast Altitude to a float")

						formattedGpsString = "{},{},{},{}".format(time, latitude, longitude, altitude)
					else:
						log("GPS", 'Line length unexpected - incomplete packet: ' + str(gpsSplit))
						print('Line length unexpected - incomplete packet: ' + str(gpsSplit))
				except:
					print('Format data: no valid GPS string - exception caught')
					self.exceptionDictionary('LINE_HANDLING', 10)
					print('offending string: ' + str(gpsString))
					formattedGpsString = ",,,"
					validGpsData = False

		except:
			self.exceptionDictionary("GPS_HANDLING", 16)

		return formattedGpsString, validGpsData

	def gpsSerialInput(self):
		messageReceived = "NO_GPS_DATA\n"
		serialInput = ""
		readAttempts = 5
		retriesToWait = 5

		try:
			self.gpsSerialPort.flushOutput()
			self.gpsSerialPort.flushInput()
			time.sleep(.6)

			while (readAttempts > 0 and retriesToWait > 0):
				if (self.gpsSerialPort.inWaiting() > 0):  # If there's a buffer for us to read
					serialInput = self.gpsSerialPort.readline(512)
					if (serialInput[:6] == r"$GPGGA"):  # Makes sure this is the line we want
						self.foundCorrectUSB = True
						break  # This is our stop
					else:
						# print("Discarding unused data: " + serialInput)
						log("GPS", serialInput)
						serialInput = ""  # This is not the data we're looking for
						readAttempts -= 1
				else:
					print('GPS serial port is not ready')
					time.sleep(0.2)
					retriesToWait -= 1
		except:
			print("Exception thrown while trying to read GPS serial input")
			self.exceptionDictionary('GPS_RECEIVE', 3)
			if not (self.foundCorrectUSB):
				self.switchUSB()  # REVIEW THIS LATER

		if (readAttempts > 0 and retriesToWait > 0):  # We found what we wanted
			messageReceived = serialInput

		elif not (self.foundCorrectUSB):
			self.switchUSB()

		log("GPS", messageReceived)
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
			self.exceptionDictionary('RADIO_RECEIVE', 2)

		return serialInput

	def sendSerialOutput(self, line):
		try:
			print("Outbound: " + RADIO_CALLSIGN + "," + str(line) + ",END_TX\n")
			self.radioSerialPort.write(RADIO_CALLSIGN + "," + str(line) + ",END_TX\n")
		except:
			print("Unable to write to radio serial port on " + self.RADIO_SERIAL_PORT)
			self.exceptionDictionary('RADIO_TRANSMIT', 1)

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
			if not self.balloonReleaseActivated:
				GPIO.setmode(GPIO.BCM)
				GPIO.setwarnings(False)
				GPIO.setup(23, GPIO.OUT)

				if not self.brmRecorded:
					try:
						self.recordVideo(180)

						self.brmRecorded = True

						self.sendSerialOutput('ack,BRM_RECORDING_STARTED')

						GPIO.output(23, 1)
					except:
						print("Unable to start recording!")
						self.exceptionDictionary('VIDEO_RECORD', 11)

				GPIO.output(23, 1)

				self.balloonReleaseActivated = True

		except:
			log("SCRIPT", "Unable to release the balloon. Unknown exception occurred")
			self.radioSerialOutput("error,release_balloon_cmd_failed")
			self.exceptionDictionary('BALLOON_RELEASE', 12)


	def brmReset(self):
		try:
			if (self.balloonReleaseActivated):
				GPIO.setmode(GPIO.BCM)
				GPIO.setup(23, GPIO.OUT)
				GPIO.setwarnings(False)

				GPIO.output(23, 0)

				self.balloonReleaseActivated = False

		except:
			self.exceptionDictionary('BRM_RESET', 13)



	def snapShot(self, numberOfPhotosToBurst = 5):
		try:
			currTimeInSeconds = time.mktime(datetime.datetime.now().timetuple())
			timeLeftToRecord = self.videoEndTime - currTimeInSeconds

			photosTaken = 0

			print("Video time left: " + str(timeLeftToRecord))

			if ((timeLeftToRecord) < 0):
				print("Attempting to initialize camera")
				cap = cv2.VideoCapture(0)
				time.sleep(1)
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
			self.exceptionDictionary('SNAPSHOT', 0)

	def recordVideo(self, duration):
		try:

			currTimeInSeconds = time.mktime(datetime.datetime.now().timetuple())
			timeLeftToRecord = self.videoEndTime - currTimeInSeconds

			print("Video time left: " + str(timeLeftToRecord))

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
		except:
			self.exceptionDictionary('VIDEO_RECORD', 11)

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
			self.exceptionDictionary('SWITCH_USB', 14)

	def getDiskSpace(self):
		output = subprocess.check_output(['df', '-h'])
		return str(output.split()[10])

	def exceptionDictionary(self, error, bitShifter):
		try:
			log("EXCEPTION", exceptionDictionary[error])
			self.EXCEPTION_SUM = self.EXCEPTION_SUM + (1 << int(bitShifter))
		except:
			log("EXCEPTION", 'Exception while attempting to find exception')
			print("Exception in ExceptionLoggingFunction")



def log(logType, line):
	logFile = open(logFileLocationDictionary[logType], 'a')
	logFile.write(str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ": " + line + "\n")

	logFile.close


if __name__ == '__main__':
	EXCEPTION_SUM = 0

	while(True):
		try:
			runScript = balloonScript(EXCEPTION_SUM)
		except:
			print("\n\nProgram terminated, starting again\n\n")
			time.sleep(5)
