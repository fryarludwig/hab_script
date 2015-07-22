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
import math
import smbus
import os
import numpy
import RPi.GPIO as GPIO
import time
import cv2
import datetime

RADIO_CALLSIGN = "hab"
RADIO_BAUDRATE = 9600

GPS_BAUDRATE = 4800

GPS_LOG_FILE_LOCATION = r"/home/pi/hab_script/logData/gps_log.txt"
RADIO_LOG_FILE_LOCATION = r"/home/pi/hab_script/logData/radio_log.txt"
SCRIPT_LOG_FILE_LOCATION = r"/home/pi/hab_script/logData/balloon_script_log.txt"


TIME_TO_DRIVE_BRM = 10

RELEASE_BALLOON_ALTITUDE = 21945    # 72,000 feet

VALID_CAMERA = True

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

moment = [None, None, None]


try:
        cap = cv2.VideoCapture(0)
        fourcc = cv2.VideoWriter_fourcc('I', 'Y', 'U', 'V')
        out = cv2.VideoWriter('output.avi', fourcc, 25.0, (640, 480))
except:
        VALID_CAMERA = False
'''
if cap.isOpened() == False:
	cap.open(0)
'''
class balloonScript():
	def __init__(self):
                self.foundCorrectUSB = False
                self.RADIO_SERIAL_PORT = "/dev/ttyUSB1"
                self.GPS_SERIAL_PORT = "/dev/ttyUSB0"
                
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
			self.fTrpi = open('/home/pi/hab_script/sensorData/temp_raspi.txt', 'a')
		except:
			self.fTrpi = open('/home/pi/hab_script/sensorData/temp_raspi.txt', 'w')
		try:
			self.fText = open('/home/pi/hab_script/sensorData/temp_external.txt', 'a')
		except:
			self.fText = open('/home/pi/hab_script/sensorData/temp_external.txt', 'w')
		try:
			self.fTbat = open('/home/pi/hab_script/sensorData/temp_batteries.txt', 'a')
		except:
			self.fTbat = open('/home/pi/hab_script/sensorData/temp_batteries.txt', 'w')
		try:
			self.fVbat = open('/home/pi/hab_script/sensorData/voltage_batteries.txt', 'a')
		except:
			self.fVbat = open('/home/pi/hab_script/sensorData/voltage_batteries.txt', 'w')
		try:
			self.fRH = open('/home/pi/hab_script/sensorData/humidity.txt', 'a')
		except:
			self.fRH = open('/home/pi/hab_script/sensorData/humidity.txt', 'w')
		try:
			self.fAcc = open('/home/pi/hab_script/sensorData/accelerometer.txt', 'a')
		except:
			self.fAcc = open('/home/pi/hab_script/sensorData/accelerometer.txt', 'w')

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
		while(True):
			# send			
			try:
                                time.sleep(0.5)
				gpsMessage, validGpsData = self.processGpsData(self.gpsSerialInput())
				sensorData, validSensorData = self.getSensorData()

				if (validGpsData or validSensorData):
					self.sendSerialOutput("data," + gpsMessage + sensorData)

			except:
				print("Main loop handled uncaught exception. Exiting now.")
				return 0

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

	def processCommand(self, command):
		if (command == "ARM_BRM"):
			self.balloonReleaseArmed = True
			self.sendSerialOutput("ack,BRM_ARMED")
			print("ARMING BALLOON")
		elif (command == "DISARM_BRM"):
			self.balloonReleaseArmed = False
			self.sendSerialOutput("ack,BRM_DISARMED")
			print("DISARMING BALLOON")
		elif (command == "SSAG_RELEASE_BALLOON" and
				self.balloonReleaseArmed):
			self.releaseBalloon()
			print("BALLOON RELEASE ACTIVATED")
			self.sendSerialOutput("ack,BRM_ACTIVATED")
		elif (command == "RESET_BRM"):
			self.balloonReleaseArmed = False
			self.brmReset()
			self.sendSerialOutput("ack,BRM_RESET")
			print("BALLOON RELEASE RESET")
		elif (command == "SNAPSHOT"):
                        self.snapShot()
                        self.sendSerialOutput("ack,SNAPSHOT_TAKEN," + str(self.snapCount))
                        print("SNAPSHOT " + str(snapCount) + " TAKEN")
                        snapCount = snapCount + 1
		else:
			print("COMMAND NOT RECOGNIZED")
			self.sendSerialOutput("ack,NO_OP")

	def getSensorData(self):
                validSensorData = 6
                
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
			self.fTrpi = open('/home/pi/hab_script/sensorData/temp_raspi.txt', 'a')
			self.fTrpi.write(str(VTrpi) + ' ' + calculatedPiTemp + '\n')
			self.fTrpi.close()
		except:
                        validSensorData -= 1
			calculatedPiTemp = "NO_VAL"

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
			calculatedExternalTemp = "NO_VAL"

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
			calculatedBatteryTemp = "NO_VAL"

		try:
			rawValBatteryVoltage = bus.read_byte_data(address, registerVbat)
			#calculatedVoltageBattery = "%4.1f" % (.05155 * rawValBatteryVoltage + .18659)
                        calculatedVoltageBattery = "%4.1f" % (((10.08 + 30.05) / 10.08) * rawValBatteryVoltage * 3.3 / 255)
			self.fVbat = open('/home/pi/hab_script/sensorData/voltage_batteries.txt', 'a')
			self.fVbat.write(str(rawValBatteryVoltage) + ' ' + calculatedVoltageBattery + '\n')
			self.fVbat.close()
		except:
                        validSensorData -= 1
			calculatedVoltageBattery = "NO_VAL"

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
			calculatedRHValue = "NO_VAL"

		try:
                        i = 0
			rawAccelX = (bus.read_byte_data(address, registerAccX))
			rawAccelY = (bus.read_byte_data(address, registerAccY))
			rawAccelZ = (bus.read_byte_data(address, registerAccZ))
			positionZero = [128,128,152]
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
			calculatedMagnitude = "NO_VAL"


		return ",{},{},{},{},{},{}".format(calculatedPiTemp, calculatedExternalTemp,
							calculatedBatteryTemp, calculatedVoltageBattery,
							calculatedRHValue, calculatedMagnitude), (validSensorData > 0)

	def processAltitude(self, currAlt):  
                try:
                        if (currAlt > RELEASE_BALLOON_ALTITUDE):
                                numOfDataPoints = len(self.altitudeDataList)

                                self.altitudeDataList.append(currAlt)

                                if (numOfDataPoints > 3):                        # We need 3 consecutive data points
                                        logScript("3 heights above target: " + str(self.altitudeDataList))
                                        self.releaseBalloon()
                        else:
                                self.altitudeDataList.clear()
                                
                except:
                        print("Error in calculating altitude change")


	def processGpsData(self, gpsString):
		validGpsData = False
		formattedGpsString = 'NO_VAL,NO_VAL,NO_VAL,NO_VAL'
                
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
							self.processAltitude(float(altitude))
						except:
							print("Unable to cast Altitude to a float")

					formattedGpsString = "{},{},{},{}".format(time, latitude, longitude, altitude)
				else:
                                        logGpsData('Line length unexpected - incomplete packet: ' + str(gpsSplit))
					print('Line length unexpected - incomplete packet: ' + str(gpsSplit))
			except:
				print('Format data: no valid GPS string - exception caught')
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
			if not (self.foundCorrectUSB):
                                self.switchUSB() # REVIEW THIS LATER

		if (retries > 0 and iterationsToWait > 0):  # We found what we wanted
			messageReceived = serialInput
			self.foundCorrectUSB = True
			
		elif not (self.foundCorrectUSB):
                        self.switchUSB()
                        print('Switching USB')

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

		return serialInput


	def sendSerialOutput(self, line):
		try:
			self.radioSerialPort.write(RADIO_CALLSIGN + "," + str(line) + ",END_TX\n")
		except:
			print("Unable to write to radio serial port on " + self.RADIO_SERIAL_PORT)

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
                        print('Opened GPS port ' + self.GPS_SERIAL_PORT + ' at baud ' + str(GPS_SERIAL_PORT))
		except:
			print("Failed to open the GPS serial port")

	def releaseBalloon(self):
                try:
                        GPIO.setmode(GPIO.BCM)
                        GPIO.setwarnings(False)
                        GPIO.setup(23, GPIO.OUT)
                        
                        GPIO.output(23, 0)

                        zero = time.time()
                        
                        while time.time() - zero < TIME_TO_DRIVE_BRM:
                                try:
                                        ret, frame = cap.read()
                                        out.write(frame)
                                        cv2.waitKey(1)
                                except:
                                        pass
                                GPIO.output(23, 0)

                        self.balloonReleaseActivated = True
                        
                except:
                        logScript("Unable to release the balloon. Unknown exception occurred")
                        self.radioSerialOutput("error,release_balloon_cmd_failed")

                '''
		NOTES
		
		Orange wire must be connected to pin 5 of MOSFET
		Yellow wire must be connected to pin 7 of MOSFET
		If raspi pin is high, motor will retract
		If raspi pin is low, motor will expand
		'''

	def brmReset(self):
                try:
                        GPIO.setmode(GPIO.BCM)
                        GPIO.setup(23, GPIO.OUT)
                        GPIO.setwarnings(False)
                        
                        GPIO.output(23, 1)
                        
                        zero = time.time()
                        
                        while time.time() - zero < TIME_TO_DRIVE_BRM:
                                try:
                                        ret, frame = cap.read()
                                        out.write(frame)
                                        cv2.waitKey(1)
                                except:
                                        pass
                                GPIO.output(23, 1)
                                
                        self.balloonReleaseActivated = False
                        
                except:
                        logScript("Unable to release the balloon. Unknown exception occurred")
                        self.radioSerialOutput("error,brm_reset_failed")

        
	def snapShot(self):
                self.snapCount = 0

                while os.path.isfile('/home/pi/hab_script/snapshots/snapshot' + str(self.snapCount) + '.png') == True:
                        self.snapCount = self.snapCount + 1

                ret, frame = cap.read()
                cv2.imwrite('/home/pi/hab_script/snapshots/snapshot' + str(self.snapCount) + '.png', frame)

        def switchUSB(self):
                if self.RADIO_SERIAL_PORT == "/dev/ttyUSB1":
                        self.RADIO_SERIAL_PORT = "/dev/ttyUSB0"
                        self.GPS_SERIAL_PORT = "/dev/ttyUSB1"
                else:
                        self.RADIO_SERIAL_PORT = "/dev/ttyUSB1"
                        self.GPS_SERIAL_PORT = "/dev/ttyUSB0"


                print('Switching USB ports')
                self.radioSerialPort.close()
                self.gpsSerialPort.close()

                time.sleep(1)
                
                self.openRadioSerialPort()
                self.openGpsSerialPort()
        
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
