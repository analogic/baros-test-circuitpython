import time
import board

i2c = board.I2C()

print("starting")

while not i2c.try_lock():
    pass

print("cont...")

try:
    while True:
        print("I2C addresses found:", [hex(device_address)
              for device_address in i2c.scan()])
        time.sleep(2)
 
finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
    i2c.unlock()