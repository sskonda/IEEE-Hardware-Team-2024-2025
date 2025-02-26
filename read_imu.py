#!/bin/python3

import pigpio
import time
import struct


buffer = memoryview

BUS=1

MPU_6050_I2C_ADDR=0b1101001

RUNTIME=60.0

# Registers.

MPU_6050_WHO_AM_I    = 0x75 # R/W
MPU_6050_ACCEL_XOUT_H = 0x3B # R
MPU_6050_ACCEL_XOUT_L = 0x3C # R
MPU_6050_ACCEL_YOUT_H = 0x3D # R
MPU_6050_ACCEL_YOUT_L = 0x3E # R
MPU_6050_ACCEL_ZOUT_H = 0x3F # R
MPU_6050_ACCEL_ZOUT_L = 0x40 # R
MPU_6050_TEMP_OUT_H  = 0x41 # R
MPU_6050_TEMP_OUT_L  = 0x42 # R
MPU_6050_GYRO_XOUT_H = 0x43 # R
MPU_6050_GYRO_XOUT_L = 0x44 # R
MPU_6050_GYRO_YOUT_H = 0x45 # R
MPU_6050_GYRO_YOUT_L = 0x46 # R
MPU_6050_GYRO_ZOUT_H = 0x47 # R
MPU_6050_GYRO_ZOUT_L = 0x48 # R

pi=pigpio.pi() # open local Pi

h = pi.i2c_open(BUS, MPU_6050_I2C_ADDR)
read = 0

if h >= 0: # Connected OK?
   # Initialise MPU_6050.
   start_time = time.time()

   while (time.time()-start_time) < RUNTIME:

      # > = big endian
      print(f"Who Am I {pi.i2c_read_byte_data(h, MPU_6050_WHO_AM_I)}")

      (s, b) = pi.i2c_read_i2c_block_data(h, MPU_6050_TEMP_OUT_H, 6 + 2 + 6)

      if s >= 0:
         (accel_x, accel_y, accel_z, t, gyro_x, gyro_y, gyro_z) = struct.unpack('>7h', buffer(b))
         t = 35 + ((t + 13200) / 280.0)
         print("{:.1f} {} {} {}".format(t, gyro_x, gyro_y, gyro_z))
         read += 1

   pi.i2c_close(h)

pi.stop()

print(read, read/RUNTIME)
