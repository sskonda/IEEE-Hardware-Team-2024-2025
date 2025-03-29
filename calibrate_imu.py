#!/bin/python3

import numpy as np
import pigpio
import time
import struct
from tqdm import tqdm
import numpy as np
from tqdm import tqdm

import numpy as np
from tqdm import tqdm

def fit_ellipsoid(data):
    # Data is an N x 3 numpy array of floats to fit
    x, y, z = data[:, 0], data[:, 1], data[:, 2]
    
    # Construct the design matrix
    D = np.column_stack([x**2, y**2, z**2, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z, np.ones_like(x)])
    
    # Solve the least squares problem
    _, _, V = np.linalg.svd(D, full_matrices=False)
    coeffs = V[-1, :]
    
    return coeffs  # Returns the 10 coefficients of the fitted ellipsoid equation

def undistort(data, coeffs):
    # Extract ellipsoid parameters
    A, B, C, D, E, F, G, H, I, J = coeffs
    
    # Compute the offset (center of the ellipsoid)
    M = np.array([[2*A, D, E], [D, 2*B, F], [E, F, 2*C]])
    center = -np.linalg.solve(M, np.array([G, H, I]))
    
    # Translate data to center the ellipsoid at the origin
    translated_data = data - center
    
    # Compute the transformation matrix (rotation and scaling)
    eigenvalues, eigenvectors = np.linalg.eig(M)
    scale_factors = np.sqrt(1 / np.abs(eigenvalues))  # Scale factors to make the ellipsoid a unit sphere
    
    # Align axes by rotating data to the principal axes
    aligned_data = (eigenvectors.T @ translated_data.T).T
    
    # Apply scaling correction
    scaled_data = aligned_data * scale_factors
    
    # Rotate back to the original orientation
    corrected_data = (eigenvectors @ scaled_data.T).T
    
    return corrected_data
   


if __name__ == '__main__':
   NUM_POSES = 15
   NUM_SAMPLES = 10000

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

      calibration_data = np.empty((NUM_SAMPLES * NUM_POSES, 6), dtype=np.int16)

      # Initialise MPU_6050.
      start_time = time.time()
      print(f"Who Am I {pi.i2c_read_byte_data(h, MPU_6050_WHO_AM_I)}")

      n = 0
      while n < NUM_POSES:
         input(f"Press enter once the robot is stable...")
         print(f"Recording {n}...")
         for i in tqdm(range(NUM_SAMPLES)):
            (s, b) = pi.i2c_read_i2c_block_data(h, MPU_6050_ACCEL_XOUT_H, 6 + 2 + 6)

            if s >= 0:
               # > = big endian
               idx = n * NUM_SAMPLES + i
               calibration_data[idx] = struct.unpack('>3hxx3h', b)
            else:
               print("Failed...")
         if input("Accept? (Y/n)") == 'n':
            continue
         else:
            n += 1

      calib_mean = np.mean(calibration_data, axis=0)
      print(calib_mean[:3])
      print(calib_mean[3:])

      np.save("imu_calibration_data", calibration_data)

   pi.stop()

