import struct
import pigpio
import time
from read_whoami import WHO_AM_I
from robot.components.component import Component
import numpy as np

DATA_ADDRESS = 0x3B
WHO_AM_I_ADDRESS = 0x75

WHO_AM_I = 0x70


class I2C_IMU(Component):
    def __init__(self, device_address=0x68, bus=1):
        super().__init__()
        self.device_address = device_address
        self.bus = bus

        calibation_data = np.load('imu_calibration_data.npy')
        self.calibration_offset = np.mean(calibation_data, axis=0)
        self.calibration_offset[:3] *= 386.0885826772 / 2**14
        self.calibration_offset[3:] /= 131.0
        
        self.acceleration = np.array([0.0, 0.0])
        self.velocity = np.array([0.0, 0.0])
        self.position = np.array([0.0, 0.0])

        self.angular_velocity = np.array([0.0])
        self.orientation = np.array([0.0])

    def _init(self, pi):
        self.pi = pi
        self.handle = pi.i2c_open(self.bus, self.device_address)
        
        if self.handle < 0 or self.pi.i2c_read_byte_data(self.handle, WHO_AM_I_ADDRESS) != WHO_AM_I:
            # Not the correct device
            return False

        self.last_tick = pi.get_current_tick()
        self.acceleration, self.angular_velocity = self.__read_device()
        
        return True 
    
    def __read_device(self):
        (s, b) = self.pi.i2c_read_i2c_block_data(self.handle, DATA_ADDRESS, 6 + 2 + 6)
        acceleration = np.array([0.0, 0.0])
        angular_velocity = np.array([0.0])
        if s >= 0:
            data = np.array([val for val in struct.unpack('>3hxx3h', b)])
            acceleration = -data[[1, 2]] * (386.0885826772 / 2**14)  # in/s^2
            acceleration -= self.calibration_offset[[1, 2]]
            angular_velocity = data[3] / 131.0  # deg/s
            angular_velocity -= self.calibration_offset[3]
        return (acceleration, angular_velocity)


    def get_acceleration(self):
        return self.acceleration

    def get_velocity(self):
        return self.velocity
    
    def get_position(self):
        return self.position

    def get_angular_velocity(self):
        return self.angular_velocity

    def get_orientation(self):
        return self.orientation

    def update(self):
        tick = self.pi.get_current_tick()
        (s, b) = self.pi.i2c_read_i2c_block_data(self.handle, DATA_ADDRESS, 6 + 2 + 6)
        self.acceleration, self.angular_velocity = self.__read_device()
            
        dt = pigpio.tickDiff(self.last_tick, tick) / 1_000_000.0

        self.position += (self.velocity + 0.5 * self.acceleration) * dt # Trapezoidal integration
        self.velocity += self.acceleration * dt
        
        self.orientation += self.angular_velocity * dt

        self.last_tick = tick 

    def release(self):
        self.pi.i2c_close(self.handle)
    