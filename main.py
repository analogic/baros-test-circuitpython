import time
import board

from dps368 import DPS368
from ms5611 import MS5611

print("starting")

i2c = board.I2C()

try:
    print("init")
    while not i2c.try_lock():
        print ('locking')
        pass
    dps368 = DPS368()
    i2c.unlock()

    while not i2c.try_lock():
        print ('locking')
        pass
    ms5611 = MS5611()
    ms5611.initialize()
    i2c.unlock()

    while True:
        while not i2c.try_lock():
            print ('locking')
            pass
        dps368result = dps368.measurePressureOnce()
        i2c.unlock()

        while not i2c.try_lock():
            print ('locking')
            pass
        ms5611.update()
        i2c.unlock()

        ms5611result = ms5611.returnPressure()*100

        print(str(ms5611result) + "\t" + str(dps368result))

 
finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
    i2c.unlock()