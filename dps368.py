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

    bus = board.I2C()
    address = 0x76
    kP = 1040384
    kT = 1040384

    def __init__(self, address=0x76):
        self.address = address

        self.__correctTemperature()
        self.__setOversamplingRate()

    def write_byte_data(self, addr, register, data):
        payload = bytearray([register])
        if (data):
            payload = bytearray([register, data])
        self.bus.writeto(addr, payload)

    def read_byte_data(self, addr, registry):
        registry = bytearray([registry])
        result = bytearray(1)
        self.bus.writeto_then_readfrom(addr, registry, result)
        return result[0]

    def __correctTemperature(self):
        """Correct temperature.
        DPS sometimes indicates a temperature over 60 degree Celsius
        although room temperature is around 20-30 degree Celsius.
        Call this function to fix.
        """
        # Correct Temp
        self.write_byte_data(self.address, 0x0E, 0xA5)
        self.write_byte_data(self.address, 0x0F, 0x96)
        self.write_byte_data(self.address, 0x62, 0x02)
        self.write_byte_data(self.address, 0x0E, 0x00)
        self.write_byte_data(self.address, 0x0F, 0x00)

    def __setOversamplingRate(self):
        """Set oversampling rate.
        Pressure measurement rate    :  4 Hz
        Pressure oversampling rate   : 64 times
        Temperature measurement rate :  4 Hz
        Temperature oversampling rate: 64 times
        """
        # Oversampling Rate Setting (64time)
        self.write_byte_data(self.address, 0x06, 0x26)
        self.write_byte_data(self.address, 0x07, 0xA6)
        self.write_byte_data(self.address, 0x08, 0x07)
        # Oversampling Rate Configuration
        self.write_byte_data(self.address, 0x09, 0x0C)

    def __getRawPressure(self):
        """Get raw pressure from sensor.
        Returns:
            int: Raw pressure
        """
        p1 = self.read_byte_data(self.address, 0x00)
        p2 = self.read_byte_data(self.address, 0x01)
        p3 = self.read_byte_data(self.address, 0x02)
        p = (p1 << 16) | (p2 << 8) | p3
        p = getTwosComplement(p, 24)
        return p

    def __getRawTemperature(self):
        """Get raw temperature from sensor.
        Returns:
            int: Raw temperature
        """
        t1 = self.read_byte_data(self.address, 0x03)
        t2 = self.read_byte_data(self.address, 0x04)
        t3 = self.read_byte_data(self.address, 0x05)
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
        src13 = self.read_byte_data(self.address, 0x13)
        src14 = self.read_byte_data(self.address, 0x14)
        src15 = self.read_byte_data(self.address, 0x15)
        src16 = self.read_byte_data(self.address, 0x16)
        src17 = self.read_byte_data(self.address, 0x17)
        src18 = self.read_byte_data(self.address, 0x18)
        src19 = self.read_byte_data(self.address, 0x19)
        src1A = self.read_byte_data(self.address, 0x1A)
        src1B = self.read_byte_data(self.address, 0x1B)
        src1C = self.read_byte_data(self.address, 0x1C)
        src1D = self.read_byte_data(self.address, 0x1D)
        src1E = self.read_byte_data(self.address, 0x1E)
        src1F = self.read_byte_data(self.address, 0x1F)
        src20 = self.read_byte_data(self.address, 0x20)
        src21 = self.read_byte_data(self.address, 0x21)
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
        src10 = self.read_byte_data(self.address, 0x10)
        src11 = self.read_byte_data(self.address, 0x11)
        src12 = self.read_byte_data(self.address, 0x12)
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

        t = self.calcScaledTemperature()
        temperature = self.calcCompTemperature(t)

        return temperature

    def measurePressureOnce(self):
        """Measure compensated pressure once.
        Returns:
            float:One Compensated pressure value [Pa]
        """

        p = self.calcScaledPressure()
        t = self.calcScaledTemperature()
        pressure = self.calcCompPressure(p, t)

        return pressure


    def measureBothOnce(self):
        """ measures compensated temperature and compensated pressure once
        Returns:
            
            float: Compensated Temperature 
            float: Compensated Pressure
        """
        t = self.calcScaledTemperature()
        temp = self.calcCompTemperature(t)
        p = self.calcScaledPressure()
        pressure = self.calcCompPressure(p, t)
        
        return temp, pressure
