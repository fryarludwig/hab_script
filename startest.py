import datetime






def log(type, line):
	logFile = open(logFileLocationDictionary[type], 'a')

	logFile.write(str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ": " + line + "\n")
	logFile.close









fstartup = open("/home/pi/hab_script/logData/startcount_log.txt", 'r')

for string in fstartup:
    startupLine = string.split()
    startupNumber = startupLine[2]
    print startupNumber
log("STARTUP", str(startupNumber + 1))

self.sendSerialOutput("init,STARTING_SCRIPT")

