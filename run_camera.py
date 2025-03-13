#!/bin/python3
from robot.components import Camera
import numpy as np
from visualizer import draw_indicator, SERVER, OUTPUT
from robot import DRIVE, IMU
import pigpio
 

def transform_to_position_heading(matrix):
    """
    Converts a 4x4 transformation matrix into position (x, y, z)
    and heading (yaw angle in degrees).

    Parameters:
        matrix (np.ndarray): 4x4 transformation matrix.

    Returns:
        tuple: (position, heading)
        - position: (x, y, z)
        - heading: yaw angle in degrees
    """
    if matrix.shape != (4, 4):
        raise ValueError("Input matrix must be 4x4.")

    # Extract translation vector
    position = matrix[:3, 3]
    position = np.array([position[2], position[0]])
    
    # Extract rotation matrix
    rotation_matrix = matrix[:3, :3]

    # Yaw (heading) is the rotation around the Y-axis
    yaw = np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2]) * (180.0 / np.pi)

    return (position, yaw)


def main():
    PI = pigpio.pi()
    PI.wave_clear()
    camera = Camera(0)
    camera.init(PI)
    DRIVE.init(PI)
    IMU.init(PI)
    SERVER.start()

    try:
        while True:
            DRIVE.update()
            IMU.update()
            DRIVE.current_heading[:] = IMU.current_heading[:]
            
            detections= camera.detect_apriltag()
            bg = np.zeros((500, 500, 3), np.uint8)
            origin = draw_indicator(bg, (0, 0), 0, color=(0, 255, 0))# green
            
            if not detections:
                OUTPUT.write(origin)
                continue

            for detection in detections:
                tag_id = detection["id"]  # get ID from detection 
                print(f"Detected AprilTag ID: {tag_id}")
            
                tag_pose_matrix = camera.calculate_pose_matrix()
                camera_pose = np.linalg.inv(tag_pose_matrix)

                camera_position, camera_heading = camera.get_world_positon(camera_pose, tag_id) or (None, None, None)
                #position, heading = transform_to_position_heading(camera_pose)

                if camera_position is not None and camera_heading is not None:
                    print(f"Pose matrix for tag {tag_id}: \n{camera_pose}")
                    print("Position:", camera_position)
                    print("Heading (yaw):", camera_heading, "degrees")
                    
                    motor_position = DRIVE.current_position
                    motor_heading = DRIVE.current_heading
                    
                    position_diff, heading_diff = camera.compare_camera_motor_position(
                        camera_position, camera_heading, motor_position, motor_heading
                    )

                    if position_diff is not None and heading_diff is not None:
                        print(f"Position Difference: {position_diff} inches")
                        print(f"Heading Difference: {heading_diff} degrees")

                    robot_drive = draw_indicator(origin, motor_position, motor_heading, color=(255, 0, 0)) # blue 
                    robot_camera = draw_indicator(origin, camera_position, camera_heading, color=(255, 0, 255))# purple
                    OUTPUT.write(robot_camera)

    except KeyboardInterrupt:
        pass

    finally:
        DRIVE.stop()
    

if __name__ == "__main__":
    main()
    