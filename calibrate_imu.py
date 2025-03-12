#!/bin/python3

import numpy as np
import pigpio
import time
import struct
from tqdm import tqdm

import sys

num_samples = 100000

BUS=1

MPU_6050_I2C_ADDR=0x68

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

if h >= 0: # Connected OK?
   accel_sum = np.array([0, 0, 0])
   gyro_sum = np.array([0, 0, 0])

   calibration_data = np.empty((num_samples, 6), dtype=np.int16)

   # Initialise MPU_6050.
   start_time = time.time()
   print(f"Who Am I {pi.i2c_read_byte_data(h, MPU_6050_WHO_AM_I)}")

   for i in tqdm(range(num_samples)):
      # > = big endian
      (s, b) = pi.i2c_read_i2c_block_data(h, MPU_6050_ACCEL_XOUT_H, 6 + 2 + 6)

      if s >= 0:
         data = struct.unpack('>3hxx3h', b)

         calibration_data[i] = data
         # print(f"Acceleration: {calibration_data[read, :3]}")
         # print(f"Gyro: {calibration_data[read, 3:]}")

   calib_mean = np.mean(calibration_data, axis=0)
   print(calib_mean[:3])
   print(calib_mean[3:])

   np.save("imu_calibration_data", calibration_data)
   print("Runtime:", time.time() - start_time)

   pi.i2c_close(h)

pi.stop()

