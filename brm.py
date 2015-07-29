
import time

lastBurstTime = time.time()
intervalDuration = 3

while(True):
	if (time.time() - lastBurstTime) >= intervalDuration:
		print("Taken!")
		lastBurstTime = time.time()
