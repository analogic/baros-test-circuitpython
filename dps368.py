import time
import board

def getTwosComplement(raw_val, length):
        """Get two's complement of `raw_val`.
        Args:
            raw_val (int): Raw value
            length (int): Max bit length
        Returns:
            int: Two's complement
        """
        val = raw_val
        if raw_val & (1 << (length - 1)):
            val = raw_val - (1 << length)
        return val
  
class DPS368:

    __DPS368_ADDRESS = 0x77

    
    __MS5611_ADDRESS_CSB_HIGH = 0x77
    __MS5611_DEFAULT_ADDRESS  = 0x77

    __MS5611_RA_ADC           = 0x00
    __MS5611_RA_RESET         = 0x1E

    __MS5611_RA_C0            = 0xA0
    __MS5611_RA_C1            = 0xA2
    __MS5611_RA_C2            = 0xA4
    __MS5611_RA_C3            = 0xA6
    __MS5611_RA_C4            = 0xA8
    __MS5611_RA_C5            = 0xAA
    __MS5611_RA_C6            = 0xAC
    __MS5611_RA_C7            = 0xAE

    __MS5611_RA_D1_OSR_256    = 0x40
    __MS5611_RA_D1_OSR_512    = 0x42
    __MS5611_RA_D1_OSR_1024   = 0x44
    __MS5611_RA_D1_OSR_2048   = 0x46
    __MS5611_RA_D1_OSR_4096   = 0x48

    __MS5611_RA_D2_OSR_256    = 0x50
    __MS5611_RA_D2_OSR_512    = 0x52
    __MS5611_RA_D2_OSR_1024   = 0x54
    __MS5611_RA_D2_OSR_2048   = 0x56
    __MS5611_RA_D2_OSR_4096   = 0x58

    def __init__(self, address = 0x77):
        self.bus = board.I2C()
        self.address = address

        self.__correctTemperature()
        self.__setOversamplingRate()
        
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

    def __correctTemperature(self):
        """Correct temperature.
        DPS sometimes indicates a temperature over 60 degree Celsius
        although room temperature is around 20-30 degree Celsius.
        Call this function to fix.
        """
        # Correct Temp
        self.bus.write_byte_data(self.addr, 0x0E, 0xA5)
        self.bus.write_byte_data(self.addr, 0x0F, 0x96)
        self.bus.write_byte_data(self.addr, 0x62, 0x02)
        self.bus.write_byte_data(self.addr, 0x0E, 0x00)
        self.bus.write_byte_data(self.addr, 0x0F, 0x00)

    def __setOversamplingRate(self):
        """Set oversampling rate.
        Pressure measurement rate    :  4 Hz
        Pressure oversampling rate   : 64 times
        Temperature measurement rate :  4 Hz
        Temperature oversampling rate: 64 times
        """
        # Oversampling Rate Setting (64time)
        self.bus.write_byte_data(self.addr, 0x06, 0x26)
        self.bus.write_byte_data(self.addr, 0x07, 0xA6)
        self.bus.write_byte_data(self.addr, 0x08, 0x07)
        # Oversampling Rate Configuration
        self.bus.write_byte_data(self.addr, 0x09, 0x0C)

    def __getRawPressure(self):
        """Get raw pressure from sensor.
        Returns:
            int: Raw pressure
        """
        p1 = self.bus.read_byte_data(self.addr, 0x00)
        p2 = self.bus.read_byte_data(self.addr, 0x01)
        p3 = self.bus.read_byte_data(self.addr, 0x02)
        p = (p1 << 16) | (p2 << 8) | p3
        p = getTwosComplement(p, 24)
        return p

    def __getRawTemperature(self):
        """Get raw temperature from sensor.
        Returns:
            int: Raw temperature
        """
        t1 = self.bus.read_byte_data(self.addr, 0x03)
        t2 = self.bus.read_byte_data(self.addr, 0x04)
        t3 = self.bus.read_byte_data(self.addr, 0x05)
        t = (t1 << 16) | (t2 << 8) | t3
        t = getTwosComplement(t, 24)
        return t

    def __getPressureCalibrationCoefficients(self):
        """Get pressure calibration coefficients from sensor.
        Returns:
            int: Pressure calibration coefficient (c00)
            int: Pressure calibration coefficient (c10)
            int: Pressure calibration coefficient (c20)
            int: Pressure calibration coefficient (c30)
            int: Pressure calibration coefficient (c01)
            int: Pressure calibration coefficient (c11)
            int: Pressure calibration coefficient (c21)
        """
        src13 = self.bus.read_byte_data(self.addr, 0x13)
        src14 = self.bus.read_byte_data(self.addr, 0x14)
        src15 = self.bus.read_byte_data(self.addr, 0x15)
        src16 = self.bus.read_byte_data(self.addr, 0x16)
        src17 = self.bus.read_byte_data(self.addr, 0x17)
        src18 = self.bus.read_byte_data(self.addr, 0x18)
        src19 = self.bus.read_byte_data(self.addr, 0x19)
        src1A = self.bus.read_byte_data(self.addr, 0x1A)
        src1B = self.bus.read_byte_data(self.addr, 0x1B)
        src1C = self.bus.read_byte_data(self.addr, 0x1C)
        src1D = self.bus.read_byte_data(self.addr, 0x1D)
        src1E = self.bus.read_byte_data(self.addr, 0x1E)
        src1F = self.bus.read_byte_data(self.addr, 0x1F)
        src20 = self.bus.read_byte_data(self.addr, 0x20)
        src21 = self.bus.read_byte_data(self.addr, 0x21)
        c00 = (src13 << 12) | (src14 << 4) | (src15 >> 4)
        c00 = getTwosComplement(c00, 20)
        c10 = ((src15 & 0x0F) << 16) | (src16 << 8) | src17
        c10 = getTwosComplement(c10, 20)
        c20 = (src1C << 8) | src1D
        c20 = getTwosComplement(c20, 16)
        c30 = (src20 << 8) | src21
        c30 = getTwosComplement(c30, 16)
        c01 = (src18 << 8) | src19
        c01 = getTwosComplement(c01, 16)
        c11 = (src1A << 8) | src1B
        c11 = getTwosComplement(c11, 16)
        c21 = (src1E < 8) | src1F
        c21 = getTwosComplement(c21, 16)
        return c00, c10, c20, c30, c01, c11, c21

    def __getTemperatureCalibrationCoefficients(self):
        """Get temperature calibration coefficients from sensor.
        Returns:
            int: Temperature calibration coefficient (c0)
            int: Temperature calibration coefficient (c1)
        """
        src10 = self.bus.read_byte_data(self.addr, 0x10)
        src11 = self.bus.read_byte_data(self.addr, 0x11)
        src12 = self.bus.read_byte_data(self.addr, 0x12)
        c0 = (src10 << 4) | (src11 >> 4)
        c0 = getTwosComplement(c0, 12)
        c1 = ((src11 & 0x0F) << 8) | src12
        c1 = getTwosComplement(c1, 12)
        return c0, c1

#############################################################################
    def calcScaledPressure(self):
        """Calculate scaled pressure.
        Returns:
            float: Scaled pressure
        """
        raw_p = self.__getRawPressure()
        scaled_p = raw_p / self.kP
        return scaled_p

    def calcScaledTemperature(self):
        """Calculate scaled temperature.
        Returns:
            float: Scaled temperature
        """
        raw_t = self.__getRawTemperature()
        scaled_t = raw_t / self.kT
        return scaled_t

    def calcCompTemperature(self, scaled_t):
        """Calculate compensated temperature.
        Args:
            scaled_t (float): Scaled temperature
        Returns:
            float: Compensated temperature [C]
        """
        c0, c1 = self.__getTemperatureCalibrationCoefficients()
        comp_t = c0 * 0.5 + scaled_t * c1
        return comp_t

    def calcCompPressure(self, scaled_p, scaled_t):
        """Calculate compensated pressure.
        Args:
            scaled_p (float): Scaled pressure
            scaled_t (float): Scaled temperature
        Returns:
            float: Compensated pressure [Pa]
        """
        c00, c10, c20, c30, c01, c11, c21 = self.__getPressureCalibrationCoefficients()
        comp_p = (c00 + scaled_p * (c10 + scaled_p * (c20 + scaled_p * c30))
                + scaled_t * (c01 + scaled_p * (c11 + scaled_p * c21)))
        return comp_p
    
    
    
    def measureTemperatureOnce(self):
        
        
        """Measures compensated temperature once.
        Returns:
            float:One compensated temperature value [C]
        """
        
        t= self.calcScaledTemperature()
        temperature=self.calcCompTemperature(t)
            
        return temperature
        
    def measurePressureOnce(self):
        
        
        """Measure compensated pressure once.
        Returns:
            float:One Compensated pressure value [Pa]
        """
        
        p = self.calcScaledPressure()
        t= self.calcScaledTemperature()
        pressure =self.calcCompPressure(p, t)
        
        return pressure

#############################################################################
    def read(self, registry):
        bytes = 3 # we always read 24bit value
        self.write(registry)
        ##print("reading from " + hex(self.address) + ", registry: " + hex(registry) + ", bytes: " + str(bytes))
        buffer = bytearray(3)
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
	C1 = self.read(self.__MS5611_RA_C1) #Pressure Sensitivity
	#time.sleep(0.05)
	C2 = self.read(self.__MS5611_RA_C2) #Pressure Offset
	#time.sleep(0.05)
	C3 = self.read(self.__MS5611_RA_C3) #Temperature coefficient of pressure sensitivity
	#time.sleep(0.05)
	C4 = self.read(self.__MS5611_RA_C4) #Temperature coefficient of pressure offset
	#time.sleep(0.05)
	C5 = self.read(self.__MS5611_RA_C5) #Reference temperature
	#time.sleep(0.05)
	C6 = self.read(self.__MS5611_RA_C6) #Temperature coefficient of the temperature
	self.bus.unlock()

	## Again here we are converting the 2 8bit packages into a single decimal
	self.C1 = C1[0] * 256.0 + C1[1]
	self.C2 = C2[0] * 256.0 + C2[1]
	self.C3 = C3[0] * 256.0 + C3[1]
	self.C4 = C4[0] * 256.0 + C4[1]
	self.C5 = C5[0] * 256.0 + C5[1]
	self.C6 = C6[0] * 256.0 + C6[1]

	self.update()

    def refreshPressure(self, OSR = __MS5611_RA_D1_OSR_4096):
        self.bus.try_lock()
	self.write(OSR)
	self.bus.unlock()

    def refreshTemperature(self, OSR = __MS5611_RA_D2_OSR_4096):
        self.bus.try_lock()
	self.write(OSR)
	self.bus.unlock()

    def readPressure(self):
        self.bus.try_lock()
	D1 = self.read(self.__MS5611_RA_ADC)
	self.bus.unlock()
	self.D1 = D1[0] * 65536 + D1[1] * 256.0 + D1[2]

    def readTemperature(self):
        self.bus.try_lock()
	D2 = self.read(self.__MS5611_RA_ADC)
	self.bus.unlock()
	self.D2 = D2[0] * 65536 + D2[1] * 256.0 + D2[2]

    def calculatePressureAndTemperature(self):
	dT = self.D2 - self.C5 * 2**8
	self.TEMP = 2000 + dT * self.C6 / 2**23

	OFF = self.C2 * 2**16 + (self.C4 * dT) / 2**7
	SENS = self.C1 * 2**15 + (self.C3 * dT) / 2**8

	#if (self.TEMP >= 2000):
	#    T2 = 0
	#    OFF2 = 0
	#    SENS2 = 0
	#elif (self.TEMP < 2000):
	#    T2 = dT * dT / 2**31
	#    OFF2 = 5 * ((self.TEMP - 2000) ** 2) / 2
	#    SENS2 = OFF2 / 2
	#elif (self.TEMP < -1500):
	#    OFF2 = OFF2 + 7 * ((self.TEMP + 1500) ** 2)
	#    SENS2 = SENS2 + 11 * (self.TEMP + 1500) ** 2 / 2

	#self.TEMP = self.TEMP - T2
	#OFF = OFF - OFF2
	#SENS = SENS - SENS2

	self.PRES = (self.D1 * SENS / 2**21 - OFF) / 2**15

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
