import smbus

bus = smbus.SMBus(1)
address = 0x48

registerAccX = 0xe4  # ADC CH 5
registerAccY = 0xb4  # ADC CH 6
registerAccZ = 0xf4  # ADC CH 7

moment = [None, None, None]

try:
    fAcc = open('/home/pi/hab_script/sensorData/accelerometer.txt', 'a')
except:
    fAcc = open('/home/pi/hab_script/sensorData/accelerometer.txt', 'a')


while True:
    i = 0
    rawAccelX = (bus.read_byte_data(address, registerAccX))
    rawAccelY = (bus.read_byte_data(address, registerAccY))
    rawAccelZ = (bus.read_byte_data(address, registerAccZ))
    positionZero = [128,128,152]
    rawMoment = [rawAccelX, rawAccelY, rawAccelZ]
    while i < 3:
        moment[i] = abs(rawMoment[i] - positionZero[i])
        i = i + 1
    zcalculatedMagnitude = round(((moment[0]) ** 2 + (moment[1]) ** 2 + (moment[2]) ** 2) ** (.5), 2)
    fAcc = open('/home/pi/hab_script/sensorData/accelerometer.txt', 'a')
    fAcc.write(moment[0] + ' ' + moment[1] + ' ' + moment[2] + ' ' + magnitude + '\n')
    fAcc.close()

