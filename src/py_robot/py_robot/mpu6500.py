import struct
import rclpy
import pigpio
import numpy as np
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

class MPU6500(Node):
    DATA_ADDRESS = 0x3B
    WHO_AM_I_ADDRESS = 0x75
    WHO_AM_I = 0x70

    def __init__(self):
        super().__init__('mpu6500')

        self.device_address = self.declare_parameter('device_address', 0x68).get_parameter_value().integer_value
        self.i2c_bus_id = self.declare_parameter('i2c_bus_id', 1).get_parameter_value().integer_value
        self.timer_period = self.declare_parameter('timer_period', 0.01).get_parameter_value().double_value
        self.frame_id = self.declare_parameter('frame_id', '/imu').get_parameter_value().string_value
        axis_order = self.declare_parameter('axis_order', 'xyz').get_parameter_value().string_value

        axis_order_lower = axis_order.lower()
        x_i = axis_order_lower.index('x')
        y_i = axis_order_lower.index('y')
        z_i = axis_order_lower.index('z')
        self.__axis_i = [x_i, y_i, z_i]
        self.__axis_flip = (np.array(list(axis_order))[self.__axis_i] == np.array(list('xyz'))) * 2 - 1

        # --- Precomputed, unit-consistent calibration ---
        self.angular_offset = np.array([-0.62899367, -0.92143118, -17.54510161])
        self.linear_offset = np.array([3.02625087, -0.05, -1.2875])

        # Already in SI units: rad/s and m/s²
        self.angular_scale = np.array([1.73547499e-13, 1.05332129e-12, 3.96040876e-13])
        self.linear_scale = np.array([5.66465433e-14, 4.92715111e-13, 2.20348896e-13])

        # Covariances from calibrated dataset
        self.angular_covariance = np.zeros(9)
        self.linear_covariance = np.array([
            3.5100e-04, -1.2000e-05,  1.0000e-06,
           -1.2000e-05,  2.5392e-02, -1.8900e-04,
            1.0000e-06, -1.8900e-04,  1.2870e-02
        ])

        # self.angular_scale *= (np.pi / 180.0) / 131.0
        # self.linear_scale *= 9.80665 / 16384.0

        # Setup pigpio I2C
        self.pi = pigpio.pi()
        self.i2c_handle = self.pi.i2c_open(self.i2c_bus_id, self.device_address)
        if self.i2c_handle < 0:
            raise Exception(f"Failed to open i2c device at {self.device_address}.")

        whoami = self.pi.i2c_read_byte_data(self.i2c_handle, self.WHO_AM_I_ADDRESS)
        if whoami != self.WHO_AM_I:
            raise Exception(f"Failed to validate WHO_AM_I register. (Read {whoami} Expected {self.WHO_AM_I})")

        # ROS publisher + timer
        self.publisher = self.create_publisher(Imu, '/sensors/imu', qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(self.timer_period, self._periodic)

    def __read_device(self):
        (s, b) = self.pi.i2c_read_i2c_block_data(self.i2c_handle, self.DATA_ADDRESS, 6 + 2 + 6)
        if s >= 0:
            data = np.array([val for val in struct.unpack('>3hxx3h', b)])
            linear_acceleration = ((data[:3] - self.linear_offset) * self.linear_scale)[self.__axis_i] * self.__axis_flip
            angular_velocity = ((data[3:] - self.angular_offset) * self.angular_scale)[self.__axis_i] * self.__axis_flip
            return linear_acceleration, angular_velocity
        return None

    def _periodic(self):
        data = self.__read_device()
        if data is not None:
            linear_acceleration, angular_velocity = data
            self.orientation = angular_velocity
            msg = Imu(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self.frame_id
                ),
                # orientation=
                orientation_covariance=[-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Orientation is not provided
                angular_velocity = Vector3(x=angular_velocity[0], y=angular_velocity[1], z=angular_velocity[2]),
                angular_velocity_covariance = self.angular_covariance,
                linear_acceleration = Vector3(x=linear_acceleration[0], y=linear_acceleration[1], z=linear_acceleration[2]),
                linear_acceleration_covariance = self.linear_covariance,
            )
            print(msg)
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu = MPU6500()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
