import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

GPIO.setup(23, GPIO.OUT)
i = 0
while i < 3: 
	GPIO.output(23, 1)
	print 'GPIO is high'
	time.sleep(5)
	GPIO.output(23, 0)
	print 'GPIO is low'
	time.sleep(5)

GPIO.cleanup()
