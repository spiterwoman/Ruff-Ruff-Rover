from machine import Pin, I2C
import time

# VL53L0X default address
VL53L0X_ADDR = 0x29

# Setup I2C
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

def read_distance():
    # Start measurement
    i2c.writeto_mem(VL53L0X_ADDR, 0x00, b'\x01')
    time.sleep_ms(50)

    # Read distance (2 bytes)
    data = i2c.readfrom_mem(VL53L0X_ADDR, 0x1E, 2)
    distance = (data[0] << 8) | data[1]

    return distance

while True:
    try:
        dist = read_distance()
        print("Distance:", dist, "mm")
    except Exception as e:
        print("Error:", e)

    time.sleep(0.5)