#!/bin/python3

import pigpio

BUS = 1
MPU_ADDR = 0x68  # Change to 0x69 if AD0 is high
WHO_AM_I = 0x75

pi = pigpio.pi()
h = pi.i2c_open(BUS, MPU_ADDR)

if h >= 0:
    try:
        who_am_i = pi.i2c_read_byte_data(h, WHO_AM_I)
        print(f"WHO_AM_I register: 0x{who_am_i:02X}")
    finally:
        pi.i2c_close(h)
else:
    print("Failed to open I2C bus")

pi.stop()
