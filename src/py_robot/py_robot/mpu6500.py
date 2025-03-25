import struct
import rclpy
import pathlib
import pigpio
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
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

        calibration_filepath = pathlib.Path("~/.ros/gyro_info/imu_calibration.npz")
        if calibration_filepath.exists():
            calibration = np.load(str(calibration_filepath.resolve()))
        else:
            calibration = {}
        
        # (raw - offset) * scale
        self.angular_offset = calibration['angular_offset'] if 'angular_offset' in calibration else np.zeros(3, dtype=np.float64)
        self.linear_offset = calibration['linear_offset'] if 'linear_offset' in calibration else np.zeros(3, dtype=np.float64)
        self.angular_scale = calibration['angular_scale'] if 'angular_scale' in calibration else np.ones(3, dtype=np.float64)
        self.linear_scale = calibration['linear_scale'] if 'linear_scale' in calibration else np.ones(3, dtype=np.float64)
        self.angular_covariance = calibration['angular_covariance'] if 'angular_covariance' in calibration else np.zeros(9, dtype=np.float64)
        self.linear_covariance = calibration['linear_covariance'] if 'linear_covariance' in calibration else np.zeros(9, dtype=np.float64)
        
        # Adjust scale for units
        self.angular_scale *= (np.pi / 180.0) / 131.0  # rad/s
        self.linear_scale *= 9.80665 / 16384.0  # m/s^2

        self.pi = pigpio.pi()
        self.i2c_handle = self.pi.i2c_open(self.i2c_bus_id, self.device_address)

        if self.i2c_handle < 0:
            raise Exception(f"Failed to open i2c device at {self.device_address}.")
            
        whoami = self.pi.i2c_read_byte_data(self.i2c_handle, self.WHO_AM_I_ADDRESS)
        if whoami != self.WHO_AM_I:
            raise Exception(f"Failed to validate WHO_AM_I register. (Read {whoami} Expected {self.WHO_AM_I})")
        
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
            msg = Imu(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self.frame_id
                ),
                # orientation=,
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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
