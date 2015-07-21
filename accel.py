import smbus

bus = smbus.SMBus(1)
address = 0x48

i = 0

registerAccX = 0xf4  # ADC CH 5
registerAccY = 0xb4  # ADC CH 6
registerAccZ = 0xe4  # ADC CH 7

rawAccelX = str(bus.read_byte_data(address, registerAccX))
rawAccelY = str(bus.read_byte_data(address, registerAccY))
rawAccelZ = str(bus.read_byte_data(address, registerAccZ))

moment = [None,None,None]

while True:
    i = 0
    rawAccelX = bus.read_byte_data(address, registerAccX)
    rawAccelY = bus.read_byte_data(address, registerAccY)
    rawAccelZ = bus.read_byte_data(address, registerAccZ)
    rawMoment = [rawAccelX, rawAccelY, rawAccelZ]
    #print str(rawAccelX) + ' ' + str(rawAccelY) + ' ' + str(rawAccelZ)
    positionZero = [128,128,152]
    while i < 3:
        moment[i] = abs(rawMoment[i] - positionZero[i])
        i = i + 1
    magnitude = ((moment[0]) ** 2 + (moment[1]) ** 2 + (moment[2]) ** 2) ** (.5)
    print round(magnitude, 5)
    
