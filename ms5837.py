import time
import board

class MS5837:

    __MS5837_ADDRESS_CSB_LOW  = 0x76
    __MS5837_ADDRESS_CSB_HIGH = 0x77
    __MS5837_DEFAULT_ADDRESS  = 0x76

    __MS5837_RA_ADC           = 0x00
    __MS5837_RA_RESET         = 0x1E

    __MS5837_RA_C0            = 0xA0
    __MS5837_RA_C1            = 0xA2
    __MS5837_RA_C2            = 0xA4
    __MS5837_RA_C3            = 0xA6
    __MS5837_RA_C4            = 0xA8
    __MS5837_RA_C5            = 0xAA
    __MS5837_RA_C6            = 0xAC

    __MS5837_RA_D1_OSR_256    = 0x40
    __MS5837_RA_D1_OSR_512    = 0x42
    __MS5837_RA_D1_OSR_1024   = 0x44
    __MS5837_RA_D1_OSR_2048   = 0x46
    __MS5837_RA_D1_OSR_4096   = 0x48

    __MS5837_RA_D2_OSR_256    = 0x50
    __MS5837_RA_D2_OSR_512    = 0x52
    __MS5837_RA_D2_OSR_1024   = 0x54
    __MS5837_RA_D2_OSR_2048   = 0x56
    __MS5837_RA_D2_OSR_4096   = 0x58

    def __init__(self, address = 0x76):
	self.bus = board.I2C()
	self.address = address
	self.C1 = 0
	self.C2 = 0
	self.C3 = 0
	self.C4 = 0
	self.C5 = 0
	self.C6 = 0
	self.D1 = 0
	self.D2 = 0
	self.TEMP = 0.0 # Calculated temperature
	self.PRES = 0.0 # Calculated Pressure

    def read(self, registry, size = 3):
        self.write(registry)
        ##print("reading from " + hex(self.address) + ", registry: " + hex(registry) + ", bytes: " + str(bytes))
        buffer = bytearray(size)
        self.bus.readfrom_into(self.address, buffer)
        ##print("result: ")
        ##print(buffer)
        return buffer

    def write(self, register):
        ##print("writing " + hex(register) + " to " + hex(self.address))
        self.bus.writeto(self.address, bytearray([register]))

    def initialize(self):
        self.bus.try_lock()
	## The MS6511 Sensor stores 6 values in the EPROM memory that we need in order to calculate the actual temperature and pressure
	## These values are calculated/stored at the factory when the sensor is calibrated.
	##      I probably could have used the read word function instead of the whole block, but I wanted to keep things consistent.
	C1 = self.read(self.__MS5837_RA_C1) #Pressure Sensitivity
	#time.sleep(0.05)
	C2 = self.read(self.__MS5837_RA_C2) #Pressure Offset
	#time.sleep(0.05)
	C3 = self.read(self.__MS5837_RA_C3) #Temperature coefficient of pressure sensitivity
	#time.sleep(0.05)
	C4 = self.read(self.__MS5837_RA_C4) #Temperature coefficient of pressure offset
	#time.sleep(0.05)
	C5 = self.read(self.__MS5837_RA_C5) #Reference temperature
	#time.sleep(0.05)
	C6 = self.read(self.__MS5837_RA_C6) #Temperature coefficient of the temperature
	self.bus.unlock()

	## Again here we are converting the 2 8bit packages into a single decimal
	self.C1 = C1[0] * 256.0 + C1[1]
	self.C2 = C2[0] * 256.0 + C2[1]
	self.C3 = C3[0] * 256.0 + C3[1]
	self.C4 = C4[0] * 256.0 + C4[1]
	self.C5 = C5[0] * 256.0 + C5[1]
	self.C6 = C6[0] * 256.0 + C6[1]

	self.update()

    def refreshPressure(self, OSR = __MS5837_RA_D1_OSR_4096):
        self.bus.try_lock()
	self.write(OSR)
	self.bus.unlock()

    def refreshTemperature(self, OSR = __MS5837_RA_D2_OSR_4096):
        self.bus.try_lock()
	self.write(OSR)
	self.bus.unlock()

    def readPressure(self):
        self.bus.try_lock()
	D1 = self.read(self.__MS5837_RA_ADC)
	self.bus.unlock()
	self.D1 = D1[0] * 65536 + D1[1] * 256.0 + D1[2]

    def readTemperature(self):
        self.bus.try_lock()
	D2 = self.read(self.__MS5837_RA_ADC)
	self.bus.unlock()
	self.D2 = D2[0] * 65536 + D2[1] * 256.0 + D2[2]

    def calculatePressureAndTemperature(self):
	dT = self.D2 - self.C5 * 2**8
	self.TEMP = 2000 + dT * self.C6 / 2**23

	OFF = self.C2 * 2**16 + (self.C4 * dT) / 2**7
	SENS = self.C1 * 2**15 + (self.C3 * dT) / 2**8

        # we will do only first order, since we are small int only
	#if (self.TEMP >= 2000):
	#    print("demg")
	#    print("T2: " + str(dT ** 2))
	#    print("omg")
	#    T2 = int(2 * dT * dT) >> 37
	#    OFF2 = (self.TEMP - 2000) ** 2 / 2**4
	#    SENS2 = 0
	#elif (self.TEMP < 2000):
	#    T2 = dT * dT / 2**33
	#    OFF2 = 3 * ((self.TEMP - 2000) ** 2) / 2
	#    SENS2 = 5 * ((self.TEMP - 2000) ** 2) / 2 ** 3
	#elif (self.TEMP < -1500):
	#    OFF2 = OFF2 + 7 * ((self.TEMP + 1500) ** 2)
	#    SENS2 = SENS2 + 4 * ((self.TEMP + 1500) ** 2)
	#self.TEMP = self.TEMP - T2
	#OFF = OFF - OFF2
	#SENS = SENS - SENS2

	self.PRES = (self.D1 * SENS / 2**21 - OFF) / 2**14

	self.TEMP = self.TEMP / 100 # Temperature updated
	self.PRES = self.PRES / 100 # Pressure updated

    def returnPressure(self):
	return self.PRES

    def returnTemperature(self):
	return self.TEMP

    def update(self):
	self.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready
	self.readPressure()

	self.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready
	self.readTemperature()

	self.calculatePressureAndTemperature()
